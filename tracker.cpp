// The contents of this file are in the public domain. See LICENSE_FOR_EXAMPLE_PROGRAMS.txt
/*

    This example shows how to use the correlation_tracker from the dlib C++ library.  This
    object lets you track the position of an object as it moves from frame to frame in a
    video sequence.  To use it, you give the correlation_tracker the bounding box of the
    object you want to track in the current video frame.  Then it will identify the
    location of the object in subsequent frames.

    In this particular example, we are going to run on the video sequence that comes with
    dlib, which can be found in the examples/video_frames folder.  This video shows a juice
    box sitting on a table and someone is waving the camera around.  The task is to track the
    position of the juice box as the camera moves around.
 */

#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
#include <dlib/image_processing/box_overlap_testing_abstract.h>


using namespace dlib;
using namespace std;

double
getJaccardCoefficient(double leftCol, double topRow, double rightCol, double bottomRow,
					  double gtLeftCol,	double gtTopRow, double gtRightCol, double gtBottomRow)
{
    double jaccCoeff = 0.;

    if (!(leftCol > gtRightCol ||
        rightCol < gtLeftCol ||
        topRow > gtBottomRow ||
        bottomRow < gtTopRow)
        )
    {
    	double interLeftCol = std::max<double>(leftCol, gtLeftCol);
    	double interTopRow = std::max<double>(topRow, gtTopRow);
    	double interRightCol = std::min<double>(rightCol, gtRightCol);
    	double interBottomRow = std::min<double>(bottomRow, gtBottomRow);

        const double areaIntersection = (abs(interRightCol - interLeftCol) + 1) * (abs(interBottomRow - interTopRow) + 1);
        const double lhRoiSize = (abs(rightCol - leftCol) + 1) * (abs(bottomRow - topRow) + 1);
        const double rhRoiSize = (abs(gtRightCol - gtLeftCol) + 1) * (abs(gtBottomRow - gtTopRow) + 1);

        jaccCoeff = areaIntersection / (lhRoiSize + rhRoiSize - areaIntersection);
    }
    return jaccCoeff;
};


int main(int argc, char** argv) try
{
	//	int point_x, point_y, width, height;
	double point_x, point_y, width, height;
	std::vector<rectangle> groundtruth;
	test_box_overlap boxes_overlap(0.9999999,1);


	if (argc != 2)
	{
		cout << "Call this program like this: " << endl;
		cout << "./video_tracking_ex ../video_frames <point_x> <point_y> <width-juicebox> <height-juicebox>" << endl;
		return 1;
	}

	// Get the list of video frames.
	std::vector<file> files = get_files_in_directory_tree(argv[1], match_ending(".jpg"));
	std::sort(files.begin(), files.end());
	if (files.size() == 0)
	{
		cout << "No images found in " << argv[1] << endl;
		return 1;
	}

	std::string video_path = argv[1];

	//    // Get pixel point
	//    point_x = atoi (argv[2]);
	//    point_y = atoi (argv[3]);
	//
	//    // Get Width and Height of Juicebox
	//    width = atoi (argv[4]);
	//    height = atoi (argv[5]);

	// Open the annotation file.
	const string& groundtruth_path = video_path + "/groundtruth.txt";
	FILE* groundtruth_file_ptr = fopen(groundtruth_path.c_str(), "r");
	int frame_num = 0;

	while (true) {
		// Read the annotation data.
		rectangle rect;
		const int status = fscanf(groundtruth_file_ptr, "%lf,%lf,%lf,%lf\n",
				&point_x, &point_y, &width, &height);

		if (status == EOF) {
			break;
		}

		// Increment the frame number.
		//frame_num++;
		rect.set_left(point_x);
		rect.set_top(point_y);
		rect.set_right(point_x + width);
		rect.set_bottom(point_y + height);

		groundtruth.push_back(rect);

	} // Process annotation file

	fclose(groundtruth_file_ptr);

	const rectangle initial_bb = groundtruth.at(0);

	// Load the first frame.
	array2d<unsigned char> img;
	load_image(img, files[0]);
	// Now create a tracker and start a track on the juice box.  If you look at the first
	// frame you will see that the juice box is centered at pixel point(92,110) and 38
	// pixels wide and 86 pixels tall.
	correlation_tracker tracker;
	tracker.start_track(img, initial_bb);

	// Now run the tracker.  All we have to do is call tracker.update() and it will keep
	// track of the juice box!
	image_window win;
	for (unsigned long i = 1; i < files.size(); ++i)
	{
		load_image(img, files[i]);
		tracker.update(img);

		win.set_image(img);
		win.clear_overlay();
		win.add_overlay(tracker.get_position());
		win.add_overlay(groundtruth.at(i),rgb_pixel(0,255,0));

		double l = tracker.get_position().left();
		double t = tracker.get_position().top();
		double r = tracker.get_position().right();
		double b = tracker.get_position().bottom();

		double l_gt = (double) groundtruth.at(i).left();
		double t_gt = (double) groundtruth.at(i).top();
		double r_gt = (double) groundtruth.at(i).right();
		double b_gt = (double) groundtruth.at(i).bottom();

		double jaccard = getJaccardCoefficient(l, t, r, b, l_gt, t_gt, r_gt, b_gt);

		cout << "hit enter to process next frame" << endl;
		cout << "jaccard:" << jaccard << endl;
		cin.get();
	}
}
catch (std::exception& e)
{
	cout << e.what() << endl;
}

