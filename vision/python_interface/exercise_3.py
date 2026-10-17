# Copyright 2026 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""Vision 3: classical ring detection and PnP pose estimation.

Assumptions: a fully visible upright square gate and modest camera roll.
Gate-local axes are x right, y down, z normal, origin at its centre.
localize_gate(image) follows the assignment interface: gate pose in the
OpenCV camera frame, with quaternion (w,x,y,z).

IMPORTANT DATASET CONVENTION: the supplied 65-image archive's labels are
consistent with CAMERA pose in a WORLD frame (x forward, y left, z up),
not gate pose in the camera frame as stated in the assignment PDF.
The fixed gate lies in world x=0 with centre (0,0,1.35) metres.
The CLI defaults to that label convention; --label_frame gate_camera uses
the PDF convention instead. --gate_height changes the fixed world height.
This frame interpretation was checked against this archive, not unseen data.
No label, filename, or reference pose is used to estimate an image's pose.
Corner identity uses an upright/low-roll assumption; arbitrary roll remains
ambiguous for an unmarked square. No OCR or learned corner identity is used.
"""
import argparse
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

IMAGE_WIDTH, IMAGE_HEIGHT = 1640, 1232
K = np.array([[365.77364921569824, 0, 820],
              [0, 365.773645401001, 616], [0, 0, 1]], dtype=np.float64)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)
OUTER_GATE_SIZE, INNER_GATE_SIZE = 2.7, 1.5
# Columns map optical right/down/forward axes into forward/left/up axes.
OPTICAL_TO_BODY = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
GATE_TO_WORLD = OPTICAL_TO_BODY.copy()


def camera_matrix(image):
    # Valid for uniformly/nonuniformly resized images, not cropped images.
    h, w = image.shape[:2]
    return np.diag([w / IMAGE_WIDTH, h / IMAGE_HEIGHT, 1.0]) @ K


def model_points():
    return np.array([(x * side / 2, y * side / 2, 0)
                     for side in (OUTER_GATE_SIZE, INNER_GATE_SIZE)
                     for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1))],
                    dtype=np.float64)


def order_quad(points):
    p = np.asarray(points, dtype=np.float64).reshape(4, 2)
    angles = np.arctan2(p[:, 1] - p[:, 1].mean(),
                        p[:, 0] - p[:, 0].mean())
    p = p[np.argsort(angles)]
    # Upright gate and modest camera roll: the leftmost of the two highest
    # vertices identifies top-left more reliably than min(x+y) in perspective.
    top = np.argsort(p[:, 1])[:2]
    start = top[np.argmin(p[top, 0])]
    return np.roll(p, -start, axis=0)


def quad(contour):
    perimeter = cv2.arcLength(contour, True)
    for fraction in (0.015, 0.025, 0.04, 0.06):
        approx = cv2.approxPolyDP(contour, fraction * perimeter, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            return order_quad(approx)
    return None


def detect_gate_corners(image):
    """Return eight pixel corners per ring, outer then inner, in cyclic order.

    Try saturated colour masks and grayscale masks. Require nested convex
    quadrilaterals whose geometry fits the known outer/inner size ratio.
    This is a baseline detector; tune masks using actual course images.
    """
    h, w = image.shape[:2]
    scale = min(1.0, 900.0 / max(h, w))
    small = cv2.resize(image, None, fx=scale, fy=scale)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    masks = [cv2.inRange(hsv, (0, 65, 35), (179, 255, 255))]
    for low, high in ((0, 15), (15, 40), (40, 90), (90, 135), (135, 179)):
        masks.append(cv2.inRange(hsv, (low, 65, 35), (high, 255, 255)))
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    masks.extend([binary, cv2.bitwise_not(binary)])
    candidates = []
    ratio = INNER_GATE_SIZE / OUTER_GATE_SIZE
    unit = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]], np.float32)
    for mask in masks:
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:
            continue
        hierarchy = hierarchy[0]
        for idx, contour in enumerate(contours):
            area = abs(cv2.contourArea(contour))
            if area < small.shape[0] * small.shape[1] * 0.001:
                continue
            outer = quad(contour)
            if outer is None:
                continue
            transform = cv2.getPerspectiveTransform(unit, outer.astype(np.float32))
            expected = cv2.perspectiveTransform((unit * ratio)[None], transform)[0]
            child = hierarchy[idx][2]
            while child != -1:
                inner = quad(contours[child])
                inner_area = abs(cv2.contourArea(contours[child]))
                if inner is not None and 0.07 < inner_area / area < 0.8:
                    choices = [np.roll(p, k, axis=0)
                               for p in (inner, inner[::-1]) for k in range(4)]
                    inner = min(choices, key=lambda p: np.mean((p - expected)**2))
                    mismatch = np.sqrt(np.mean(np.sum((inner - expected)**2, axis=1)))
                    relative = mismatch / np.sqrt(area)
                    if relative < 0.12:
                        pts = np.vstack([outer, inner]) / scale
                        candidates.append((relative, -area, pts))
                child = hierarchy[child][0]
    # Prefer size-ratio agreement; suppress repeated masks of the same ring.
    candidates.sort(key=lambda item: (item[0], item[1]))
    gates = []
    for _, _, points in candidates:
        if not any(np.linalg.norm(points[:4].mean(axis=0) - old[:4].mean(axis=0))
                   < 0.25 * np.sqrt(abs(cv2.contourArea(old[:4].astype(np.float32))))
                   for old in gates):
            gates.append(points)
    return gates


def estimate_pose(points, image):
    obj = model_points()
    cam = camera_matrix(image)
    ok, rvec, tvec = cv2.solvePnP(obj, points, cam, DIST_COEFFS,
                                 flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or not np.isfinite(tvec).all():
        return None
    matrix, _ = cv2.Rodrigues(rvec)
    if np.any((obj @ matrix.T + tvec.reshape(1, 3))[:, 2] <= 0):
        return None
    projected, _ = cv2.projectPoints(obj, rvec, tvec, cam, DIST_COEFFS)
    rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - points)**2, axis=1))))
    # Reject inconsistent geometry rather than inventing a pose.
    diagonal = np.linalg.norm(points[0] - points[2])
    if rms > max(4.0, diagonal * 0.04):
        return None
    qx, qy, qz, qw = Rotation.from_matrix(matrix).as_quat()
    pose = (tuple(float(x) for x in tvec.ravel()),
            (float(qw), float(qx), float(qy), float(qz)))
    return pose, rvec, tvec, rms


def localize_gate(image):
    """Required interface: list of ((x,y,z), (qw,qx,qy,qz))."""
    results = [estimate_pose(points, image) for points in detect_gate_corners(image)]
    return [result[0] for result in results if result is not None]


def camera_world_pose(rvec, tvec, gate_height=1.35):
    """Invert gate->optical PnP, then apply fixed world/body axis transforms.

    X_camera = R_cg X_gate + t_cg
    camera_position_world = gate_origin_world - R_wg R_cg.T t_cg
    R_world_body = R_wg R_cg.T R_optical_body
    """
    r_cg, _ = cv2.Rodrigues(rvec)
    position = np.array([0., 0., gate_height]) - GATE_TO_WORLD @ r_cg.T @ tvec.ravel()
    matrix = GATE_TO_WORLD @ r_cg.T @ OPTICAL_TO_BODY.T
    qx, qy, qz, qw = Rotation.from_matrix(matrix).as_quat()
    return (tuple(float(x) for x in position),
            (float(qw), float(qx), float(qy), float(qz)))


def translation_error(a, b):
    return float(np.linalg.norm(np.asarray(a) - np.asarray(b)))


def rotation_error(a, b):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    if np.linalg.norm(a) == 0 or np.linalg.norm(b) == 0:
        return float('nan')
    dot = abs(np.dot(a / np.linalg.norm(a), b / np.linalg.norm(b)))
    return float(np.degrees(2 * np.arccos(np.clip(dot, 0, 1))))


def read_labels(path):
    labels = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        values = list(map(float, line.split()))
        if len(values) != 7:
            raise ValueError(f'{path}: expected x y z qw qx qy qz')
        labels.append((tuple(values[:3]), tuple(values[3:])))
    return labels


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset_path', required=True, type=Path)
    parser.add_argument('--show', action='store_true', help='Space/any key: next; Q: stop')
    parser.add_argument('--output_dir', type=Path, help='Optional annotated images')
    parser.add_argument('--label_frame', choices=('camera_world', 'gate_camera'),
                        default='camera_world', help='Reference-label convention (see module docstring)')
    parser.add_argument('--gate_height', type=float, default=1.35,
                        help='Known fixed gate-centre world height in metres (default: 1.35)')
    args = parser.parse_args()
    images_dir = args.dataset_path / 'images'
    labels_dir = args.dataset_path / 'labels'
    if not images_dir.is_dir() or not labels_dir.is_dir():
        parser.error('dataset_path must contain images/ and labels/ folders')
    paths = sorted(p for p in images_dir.iterdir()
                   if p.suffix.lower() in ('.jpg', '.jpeg', '.png'))
    if not paths:
        parser.error('No images found in images/')
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
    print(f'Found {len(paths)} images. Processing one image at a time.', flush=True)
    print('Model: centred XY plane, x right, y down; quaternions w,x,y,z.')
    print(f'Evaluation label frame: {args.label_frame}; fixed gate height: {args.gate_height} m.')
    print('Errors use only unambiguous one-reference/one-detection images.')
    translations, rotations = [], []
    processed = detected = evaluated = 0
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            print(f'{path.name}: unreadable image; skipped', flush=True)
            continue
        processed += 1
        found = []
        canvas = image.copy()
        for points in detect_gate_corners(image):
            estimate = estimate_pose(points, image)
            if estimate is None:
                continue
            found.append(camera_world_pose(estimate[1], estimate[2], args.gate_height)
                         if args.label_frame == 'camera_world' else estimate[0])
            for i, pt in enumerate(points):
                pos = tuple(np.round(pt).astype(int))
                cv2.circle(canvas, pos, 5, (0, 255, 0), -1)
                cv2.putText(canvas, str(i), pos, cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 255), 2)
            cv2.drawFrameAxes(canvas, camera_matrix(image), DIST_COEFFS,
                              estimate[1], estimate[2], 0.5)
        detected += bool(found)
        label_path = labels_dir / (path.stem + '.txt')
        labels = read_labels(label_path) if label_path.exists() else []
        status = f'{path.name}: {len(found)} gate(s) detected'
        if len(found) == 1 and len(labels) == 1:
            te = translation_error(labels[0][0], found[0][0])
            re = rotation_error(labels[0][1], found[0][1])
            translations.append(te)
            rotations.append(re)
            evaluated += 1
            status += f'; translation={te:.3f} m; orientation={re:.2f} deg'
            status += f'; predicted xyz={np.round(found[0][0], 3)}'
        else:
            status += '; excluded from pose averages (missing/ambiguous pairing)'
        print(status, flush=True)
        cv2.putText(canvas, f'{path.name}: {len(found)} gate(s)', (15, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        if args.output_dir:
            cv2.imwrite(str(args.output_dir / (path.stem + '.jpg')), canvas)
        if args.show:
            cv2.namedWindow('Vision 3 - any key: next; Q: quit', cv2.WINDOW_NORMAL)
            cv2.imshow('Vision 3 - any key: next; Q: quit', canvas)
            if cv2.waitKey(0) & 0xff in (ord('q'), 27):
                break
    if args.show:
        cv2.destroyAllWindows()
    print(f'Processed: {processed}/{len(paths)}; images with detections: {detected}; evaluated: {evaluated}')
    if evaluated:
        print(f'Mean translation error = {np.mean(translations):.6f} metres')
        print(f'Mean orientation error = {np.mean(rotations):.6f} degrees')
        print('Means are conditional on successful one-to-one detections, not all dataset images.')
    else:
        print('No evaluable poses. Inspect images, masks, labels and model conventions.')


if __name__ == '__main__':
    main()
