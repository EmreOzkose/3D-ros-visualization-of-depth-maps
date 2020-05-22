#include "utils.h"
#include <bits/stdc++.h>

Mat read_calibration(string path){
    ifstream inFile;
    string line;

    Mat cam = Mat::zeros( 3, 3 , CV_32FC1);

    //Mat cam(3, 3, DataType<float>::type);

    inFile.open(path);

    if (!inFile) {
        cerr << "Unable to open file " << path << endl;
        exit(1);   // call system to stop
    }

    int counter_line = 0;
    while ( getline (inFile, line) ){
        int counter_word = 0;
        istringstream ss(line); 
  
        do { 
            string word; 
            ss >> word; 
            
            if (word.compare("") != 0)
            {
                cam.at<float>(counter_line, counter_word) = stof(word);
            }
            
            counter_word++;
        } while (ss); 
        counter_line++;
    }
    
    inFile.close();

	return cam;
}

Vec3i parse_arguments(int argc, char** argv){

    cout << argc << endl;
    cout << argv[0]  << "-" << argv[1] << "-" << argv[2] << endl;
    cout << "@FOO_STRING@" << endl;

	int start_frame, end_frame, rate;

    int arg_padding = 0;
    if (USE_LAUNCH_FILE) arg_padding+=2;

	if (argc == arg_padding+1){
		start_frame = 0;
		end_frame = 5;
		rate = 30;
	}
	else if(argc == arg_padding+3){
		start_frame = stoi(argv[1]);
		end_frame = stoi(argv[2]);
		rate = 30;
	}
	else{
		start_frame = stoi(argv[1]);
		end_frame = stoi(argv[2]);
		rate = stoi(argv[3]);
	}

	Vec3i u(start_frame, end_frame, rate);

	return u;
}

Mat read_image(String path, int image_color){
	Mat img;
	if (image_color == 0)
		img = imread(path, 0);
	else
		img = imread(path);

    if(img.empty())
    {
		throw("Could not read the image: " + path);
    }

	return img;
}
