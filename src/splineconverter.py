
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import BSpline
import numpy as np
import matplotlib.pyplot as plt
from skspatial.objects import Line
from scipy.spatial.transform import Rotation as R
import json
import bisect

np.set_printoptions(precision=3, suppress=True, linewidth=100)

def get_shortest_distance_from_point_to_segment(point, seg_start, seg_end):
    """
    Calculates the shortest distance from a point to a line segment defined by seg_start and seg_end.
    """
    lineseg = Line.from_points(seg_start, seg_end)
    projpt = lineseg.project_point(point)
    if projpt[0] < min(seg_start[0], seg_end[0]) or projpt[0] > max(seg_start[0], seg_end[0]) or \
       projpt[1] < min(seg_start[1], seg_end[1]) or projpt[1] > max(seg_start[1], seg_end[1]) or \
        projpt[2] < min(seg_start[2], seg_end[2]) or projpt[2] > max(seg_start[2], seg_end[2]):
        # If the projection point is outside the segment, return the distance to the nearest endpoint
        dist_to_start = np.linalg.norm(np.array(point) - np.array(seg_start))
        dist_to_end = np.linalg.norm(np.array(point) - np.array(seg_end))
        return min(dist_to_start, dist_to_end)
    else:
        # If the projection point is on the segment, return the distance to the projection point
        return lineseg.distance_point(point)
    
def get_distances_between_curve_and_segments(curve, segments):
    """
    Get a list of minimum distances between each point in the curve and the segments.
    """
    distances = []
    for point in curve:
        dist_to_segs = []
        for i in range(len(segments) - 1):
            seg_start = segments[i]
            seg_end = segments[i + 1]
            dist_to_segs.append(get_shortest_distance_from_point_to_segment(point, seg_start, seg_end))
        distances.append(min(dist_to_segs))
    return distances

def get_rmse_between_curve_and_segments(curve, segments):
    """
    Compares a subset of points on a B-spline curve to a set of line segments that represents that curve. 
    Returns a metric of how well the curve approximates the segments, right now in MSE.
    """
    distances = get_distances_between_curve_and_segments(curve, segments)
    distances_sq = [(d ** 2) for d in distances]
    return np.sqrt(np.mean(distances_sq))

def plot_segmented_curve(ax, segs, color='#009E73', label='Segmented Curve'):
    """Plots a segmented curve given a list of segment end points."""
    if segs.shape[1] != 3:
        raise ValueError("Segments must be 3D (shape: n_segments x 3)")
    line, = ax.plot(segs[:, 0], segs[:, 1], segs[:, 2], color=color, lw=2, label=label)
    return line

def get_spline_data_from_file(filepath):
    """
    Loads B-spline data from a JSON file.
    """
    with open(filepath, 'r') as f:
        data = json.load(f)
        bsplinedata = data['crv']['bsplinecrv']
        degree = bsplinedata['degree']
        control_points = np.array(bsplinedata['crv_pts']['cntrl_hull_pts']['pts'])

        return degree, control_points
    
def load_obj_spline(filepath):
    """
    Loads a B-spline from an OBJ file.
    """
    vertices = []
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith('v '):
                vertex = list(map(float, line.strip().split()[1:]))
                vertices.append(vertex)
    return np.array(vertices)

def get_segment_lengths(segments):
    """
    Calculates the lengths of each segment in a list of segments.
    """
    lengths = []
    for i in range(len(segments) - 1):
        seg_start = segments[i]
        seg_end = segments[i + 1]
        length = np.linalg.norm(seg_end - seg_start)
        lengths.append(length)
    return lengths



def segment_spline_from_files(json_filepath, obj_filepath, make_plot=False):
    # Load B-spline data from JSON file
    deg, ctrl_pts = get_spline_data_from_file(json_filepath)
    # Create standard knot vector
    knot_vector = np.arange(0, len(ctrl_pts)+deg+1)

    # Create B-spline objects from the json data
    spl_x = BSpline(knot_vector, ctrl_pts[:, 0], deg)
    spl_y = BSpline(knot_vector, ctrl_pts[:, 1], deg)
    spl_z = BSpline(knot_vector, ctrl_pts[:, 2], deg)

    # Create a parameter space
    p_end = len(knot_vector) - deg - 1
    u = np.linspace(deg, p_end, 100)
    B3_curve = np.vstack((spl_x(u), spl_y(u), spl_z(u))).T

    # Load B-spline data from OBJ file
    obj_spline = load_obj_spline(obj_filepath)

    # --------------------------------
    #        Plot the splines
    # --------------------------------
    if make_plot:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(B3_curve[:, 0], B3_curve[:, 1], B3_curve[:, 2], color='#0072B2', lw=2, label='Arbitrary BSpline Curve')

        # Plot control points and control polygon
        # ax.plot(ctrl_pts[:, 0], ctrl_pts[:, 1], ctrl_pts[:, 2], 'o--', color='#E69F00', label='Control Points')
        # ax.scatter(obj_spline[:, 0], obj_spline[:, 1], obj_spline[:, 2], s=5, color="#F529DA", label='OBJ Spline Curve')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend(loc='best')
        ax.set_aspect('equal', adjustable='box')

    # ------------------------------------------------
    #    Create and iterate on segment representation
    # ------------------------------------------------

    #initialize segs as an ndarray of the first and last points of the obj_spline
    segs = np.array([obj_spline[0,:], obj_spline[-1,:]])
    # plot_segmented_curve(ax, segs)
    # plt.draw()

    # setup MSE tracking
    judgement_points = np.linspace(deg, p_end, 20)
    judgement_curve = np.vstack((spl_x(judgement_points), spl_y(judgement_points), spl_z(judgement_points))).T

    RMSE = get_rmse_between_curve_and_segments(judgement_curve, segs)
    print(f"RMSE between the B-spline curve and the baseline segments: {RMSE:.3f}")

    # iterate until MSE is below 2mm or we have 12 segments
    i = 1
    while RMSE > 0.2 and len(segs) < 12:
        random_points = np.random.uniform(deg, p_end, 20)
        random_curve = np.vstack((spl_x(random_points), spl_y(random_points), spl_z(random_points))).T  
        random_dists = get_distances_between_curve_and_segments(random_curve, segs)

        furthest_idx = np.argmax(random_dists)
        new_point = random_curve[furthest_idx]
        # print(f"Largest distances from random points to segments: {np.max(random_dists):.3f} at {random_curve[np.argmax(random_dists)]}")

        # insert new point into segs
        existing_yvals= segs[:,1]
        idx = bisect.bisect_right(existing_yvals, new_point[1])
        segs = np.insert(segs, idx, new_point, axis=0)

        RMSE = get_rmse_between_curve_and_segments(judgement_curve, segs)
        # print(f"RMSE between the B-spline curve and the baseline segments: {RMSE:.3f}")
        i += 1

    return segs

    # seg_line_plot = plot_segmented_curve(ax, segs, color='#009E73', label=f'Discretized Branch with {i} Segments')
    # plt.draw()

        # plt.pause(1)
