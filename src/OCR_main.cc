#include "OCR_main.hh"
#include <igl/Timer.h>

OcrMain::OcrMain(string _dir)
	: dir(_dir), resize(50), showName(true), stop(false)
{
	cap = cv::VideoCapture(0);
}

bool OcrMain::SetSource(SOURCE opt, string videoName)
{
	source = opt;
	if (opt == SOURCE::VIDEO)
		cap = cv::VideoCapture(videoName);
	// else if(opt == SOURCE::CAPTUREBOARD)
	//     cap = cv::VideoCapture(0);

	if (!cap.isOpened())
	{
		if (opt == SOURCE::VIDEO)
			cout << "Cannot open " + videoName << endl;
		if (opt == SOURCE::CAPTUREBOARD)
			cout << "Cannot open captureboard" << endl;
		return false;
	}
	else
		return true;
}

bool OcrMain::Initialize(string configName)
{
	cv::waitKey(30);
	// initialize ocr
	int fps, resY, resX;
	fps = (int)cap.get(cv::CAP_PROP_FPS);
	resY = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
	resX = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
	cout << "default options: " << resX << "*" << resY << " (" << fps << "fps)" << endl;
	// load config file (problematic, seems to be the issue in captureboard)
	Config config;
	if (!config.loadConfig(dir + "/data/"+ configName))
	{
		cout << "config.yml is not open!" << endl;
		return false;
	}
	if (!config.loadBoxConfig(dir + "/data/" + config.getBoxFilename()))
	{
		cout << config.getBoxFilename() + " is not open!" << endl;
		return false;
	}
	cout << "set to: " << config.getCaptureBoardWidth() << "*" << config.getCaptureBoardHeight() << "(default fps)" << endl;
	// if(source == SOURCE::VIDEO)
	// {
		cv::Mat img;
	    for (int i = 0;i<50; i++)
	    {
	    	if (i == 30)
	    		cap.set(cv::CAP_PROP_FRAME_WIDTH, config.getCaptureBoardWidth());
	    	if (i == 40)
	    		cap.set(cv::CAP_PROP_FRAME_HEIGHT, config.getCaptureBoardHeight());
	    	// if (i == 50) /////USE DEFAULT!
	    	// 	cap.set(cv::CAP_PROP_FPS, config.getCaptureBoardFPS());

	    	if (cap.read(img))
	    	{
	    		cv::putText(img, to_string(i)+" | Resolution: " + std::to_string(img.cols) + "x" + std::to_string(img.rows),
	    					cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(60000), 1);
	    		cv::imshow("sample", img);
	    	}
	    	char key = cv::waitKey(1);
	    }
	// }
	// cv::destroyWindow("sample");
	proc.SetConfig(config);
	proc.SetBGcolor(config.getBGcolor());
	proc.SetBinThresh(config.getBinThreshold());
	roiBox = config.getRoiBoxes();
	roiNames = config.GetRoiNames();
	if (roiBox.size() != roiNames.size())
	{
		cout << "roiBox.size() != roiNames.size()" << endl;
		roiBox.resize(roiNames.size());
	}
	for(int i=0;i<roiBox.size();i++)
	{
		auto roi = roiBox[i];
		if( (0 <= roi.x) && (0 <= roi.width) && (roi.x + roi.width <= img.cols) && (0 <= roi.y) && (0 <= roi.height) && (roi.y + roi.height <= img.rows))
			continue;
		cout<<"wrong box#"+to_string(i)<<": "+roiNames[i] + " -> set to default box"<<endl;
		roiBox[i].x = 0;
		roiBox[i].width =img.cols;
		roiBox[i].y = 0;
		roiBox[i].height = img.rows;
	}
	ocr.SetOcrThreshold(config.getOcrThreshold());
	ocr.initModel();
	if (!ocr.loadTrainingData(dir + "/data/" + config.getTrainingDataFilename()))
		cout << "Failed to load OCR training data." << endl;
	else cout << "OCR training data loaded." << endl;

	return true;
}

// data [0/rotation, 1/angulation, 2/long(bed), 3/lat(bed), 4/height(bed), 5/SID, 6/FD, 7/kV(ex), 8/mAs(ex), 9/kV(flu), 10/mA(flu), 11/DAP, 12/switch]
bool OcrMain::Render(float *data)
{
	cv::Mat img;
	cap >> img;
	if(recording) {
		cv::Mat recordImg;
		cv::resize(img, recordImg, recordSize);
		time_t now = chrono::system_clock::to_time_t(chrono::system_clock::now());
        cv::putText(recordImg, string(ctime(&now)), cv::Point2i(0, 30),FONT_HERSHEY_SIMPLEX,1.f,cv::Scalar(0, 0, 255), 2);
		recorder<<recordImg;
	}
	cv::Mat imgCopy, imgResize;
	img.copyTo(imgCopy);
	std::vector<cv::Mat> digits;
	std::vector<cv::Rect> bBoxAll;
	std::vector<std::string> results;
	// bool fluoChk(false);
	for (size_t i = 0; i < roiBox.size(); i++)
	{
		cv::Mat imgCrop = img(roiBox[i]);
		cv::Mat imgBin;
		std::vector<cv::Rect> bBox;
		int top, bottom;
		std::vector<cv::Mat> digits = proc.Process(imgCrop, bBox, imgBin, top, bottom, true);
		string result("");
		bool success = ocr.recognize(digits, result);
		if (i < 4)
			results.push_back(result);
		else if (i == roiBox.size() - 1) // beamOn check
		{
			if (!result.size())
				data[i] = 0;
			// else if (fluoChk)
			// 	data[i] = 1;
			else
				data[i] = 2;
		}
		else if (success)
		{
			if(result[0]==-85) result = result.substr(1, result.size()-1);
			data[i] = atof(result.c_str());
			// if (i == 9)
				// fluoChk = success;
		}

		cv::rectangle(imgCopy, roiBox[i], CV_RGB(255, 255, 0), 1);
		if (showName)
			cv::putText(imgCopy, to_string(i) + "." + roiNames[i], roiBox[i].tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
		else
			cv::putText(imgCopy, result, roiBox[i].tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
		// cv::imshow("binary", imgBin);
		// cv::imshow("crop", imgCrop);
		// if (firstLoop)
		// 	cv::waitKey(100);
	}
	// option management
	if (results[0].substr(0, 4) == "long" || results[2].substr(0, 3) == "lat") // if first two box shows table options
	{
		if (results[1].size())
			data[2] = atof(results[1].c_str());
		if (results[3].size())
			data[3] = atof(results[3].c_str());
	}
	else
	{
		if (results[1].size())
			data[0] = atof(results[1].c_str());
		if (results[3].size())
			data[1] = atof(results[3].c_str());
		if (results[0] == "rao" || results[0] == "raao")
			data[0] = -data[0];
		if (results[2] == "cran" || results[2] == "craan")
			data[1] = -data[1];
	}

	if (source == SOURCE::VIDEO)
	{
		cv::putText(imgCopy, "frame#: " + to_string((int)cap.get(cv::CAP_PROP_POS_FRAMES)) + "/" + to_string((int)cap.get(cv::CAP_PROP_FRAME_COUNT)), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(60000), 1);
		if (cap.get(cv::CAP_PROP_POS_FRAMES) == cap.get(cv::CAP_PROP_FRAME_COUNT) - 1)
			cap.set(cv::CAP_PROP_POS_FRAMES, 0);
	}

	double ratio = (double)resize / 100.;
	cv::resize(imgCopy, imgCopy, cv::Size(img.cols * ratio, img.rows * ratio));
	
	cv::imshow("CaptureBoard", imgCopy);
	cv::createTrackbar(
		"resize", "CaptureBoard", &resize, 100, [](int pos, void *data) {}, (void *)&imgCopy);
	char key = cv::waitKey(10);
	if (key == ' ')
		showName = !showName;
	else if (key == '/')
		stop = !stop;
	else if (key == ',')
		cap.set(cv::CAP_PROP_POS_FRAMES, cap.get(cv::CAP_PROP_POS_FRAMES) - 10);
	else if (key == '.')
		cap.set(cv::CAP_PROP_POS_FRAMES, cap.get(cv::CAP_PROP_POS_FRAMES) + 10);
	if (stop)
		cap.set(cv::CAP_PROP_POS_FRAMES, cap.get(cv::CAP_PROP_POS_FRAMES) - 1);
	// char key = cv::waitKey(1);

	return (getWindowProperty("CaptureBoard", WND_PROP_AUTOSIZE) == 1);

	// firstLoop = false;

	// #ifdef VIDEO
	// if (key == ' ')
	// {
	// 	int newFrame = cap.get(cv::CAP_PROP_POS_FRAMES) + 100;
	// 	if (newFrame > cap.get(cv::CAP_PROP_FRAME_COUNT))
	// 		newFrame -= cap.get(cv::CAP_PROP_FRAME_COUNT);
	// 	cap.set(cv::CAP_PROP_POS_FRAMES, newFrame);
	// }
	// #endif
}

cv::Point p1(0, 0), p2(0, 0);
cv::Scalar rgb(0, 0, 0);
std::vector<cv::Rect> blackBox;
std::vector<cv::Rect> blackPoint;
bool click = false;
bool rclick = false;

void onMouseCropImageOCR(int event, int x, int y, int f, void *param)
{
	cv::Mat *img = (cv::Mat *)param;
	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:
		click = true;
		p1.x = x;
		p1.y = y;
		p2.x = x;
		p2.y = y;
		break;
	case cv::EVENT_LBUTTONUP:
		p2.x = x;
		p2.y = y;
		click = false;
		break;
	case cv::EVENT_MOUSEMOVE:
		if (click)
		{
			p2.x = x;
			p2.y = y;
		}
		if (rclick)
		{
			rgb = img->at<cv::Vec3b>(y, x);
			cv::imshow("color", cv::Mat(100, 100, CV_8UC3, rgb));
		}
		break;
	case cv::EVENT_RBUTTONDOWN:
		rclick = true;
		rgb = img->at<cv::Vec3b>(y, x);
		cv::imshow("color", cv::Mat(100, 100, CV_8UC3, rgb));
		break;
	case cv::EVENT_RBUTTONUP:
		rclick = false;
		break;
	default:
		break;
	}
}

void OcrMain::RenderForSetting()
{
	bool adjustHeight(false), pushROI(false);
	std::vector<cv::Point2f> roiRange(roiNames.size(), cv::Point2f(0., 0.));
	std::vector<int> roiCnt(roiNames.size(), 0);
	while (1)
	{
		cv::Mat img;
		cap >> img;
		cv::Mat imgCopy; //, imgResize;
		img.copyTo(imgCopy);
		std::vector<cv::Mat> digits;
		std::vector<cv::Rect> bBoxAll;

		if (adjustHeight)
		{
			for (int i = 0; i < roiBox.size(); i++)
			{
				cv::Mat crop = img(roiBox[i]);
				int top(0), bottom(0);
				std::vector<cv::Rect> bBox;
				proc.Process(crop, bBox, crop, top, bottom, false);
				for (auto box : bBox)
				{
					box.x += roiBox[i].x;
					box.y += roiBox[i].y;
					cv::rectangle(img, cv::Rect(box.tl(), box.br()), CV_RGB(0, 255, 0));
				}
				if (bottom - top > roiBox[i].height * 0.6)
				{
					roiCnt[i]++;
					roiRange[i].x += top;
					roiRange[i].y += bottom;
				}
			}
			for (int i = 0; i < roiBox.size(); i++)
			{
				cv::Rect box;
				if (roiCnt[i] < 10)
				{
					cv::rectangle(img, cv::Rect(roiBox[i].x, roiBox[i].y, roiBox[i].width, roiBox[i].height),
								  CV_RGB(255, 0, 0), 1);
					continue;
				}
				else
				{
					int top = floor(roiRange[i].x / (double)roiCnt[i]);
					int bottom = floor(roiRange[i].y / (double)roiCnt[i]) + 1;
					cv::rectangle(img, cv::Rect(roiBox[i].x, (roiBox[i].y + top), roiBox[i].width, (bottom - top)),
								  CV_RGB(255, 0, 0), 1);
				}
			}
		}
		for (size_t i = 0; i < roiBox.size(); i++)
		{
			// if(roiBox[i].x+roiBox[i].>img.cols)
			cv::Mat imgCrop = img(roiBox[i]);
			cv::Mat imgBin;
			std::vector<cv::Rect> bBox;
			int top, bottom;
			std::vector<cv::Mat> digits = proc.Process(imgCrop, bBox, imgBin, top, bottom, true);
			string result("");
			bool success = ocr.recognize(digits, result);

			cv::rectangle(imgCopy, roiBox[i], CV_RGB(255, 255, 0), 1);
			if (showName)
				cv::putText(imgCopy, to_string(i) + "." + roiNames[i], roiBox[i].tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
			else
				cv::putText(imgCopy, result, roiBox[i].tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

			// cv::imshow("binary", imgBin);
			// cv::imshow("crop", imgCrop);
			// if (firstLoop)
			// 	cv::waitKey(100);
		}

		if (source == SOURCE::VIDEO)
		{
			cv::putText(imgCopy, "frame#: " + to_string((int)cap.get(cv::CAP_PROP_POS_FRAMES)) + "/" + to_string((int)cap.get(cv::CAP_PROP_FRAME_COUNT)), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(60000), 1);
			if (cap.get(cv::CAP_PROP_POS_FRAMES) == cap.get(cv::CAP_PROP_FRAME_COUNT) - 1)
				cap.set(cv::CAP_PROP_POS_FRAMES, 0);
		}

		double ratio = (double)resize / 100.;
		cv::resize(imgCopy, imgCopy, cv::Size(img.cols * ratio, img.rows * ratio));
		if (click)
		{
			pushROI = true;
			cv::rectangle(imgCopy, cv::Rect(p1, p2), CV_RGB(255, 255, 0), 1);
		}
		cv::imshow("Setting", imgCopy);
		cv::setMouseCallback("Setting", onMouseCropImageOCR, &imgCopy);
		cv::createTrackbar(
			"resize", "Setting", &resize, 100, [](int pos, void *data) {}, (void *)&imgCopy);

		char key = cv::waitKey(50);
		if ((!click) && pushROI)
		{
			cv::rectangle(imgCopy, cv::Rect(p1, p2), CV_RGB(255, 255, 0), 1);

			cv::Mat crop = img(cv::Rect(p1 / ratio, p2 / ratio));
			cv::resize(crop, crop, cv::Size2i(crop.cols * 2, crop.rows * 2));
			cv::imshow("ROI - 2X view", crop);
		}
		if (key >= 0)
		{
			// use number pad(789/ for p1, 1235 for p2) to adjust box size
			if (pushROI)
			{
				if (key == 'z')
					p1.x -= 1;
				else if (key == 'c')
					p1.x += 1;
				else if (key == 's')
					p1.y -= 1;
				else if (key == 'x')
					p1.y += 1;
				else if (key == ',')
					p2.x -= 1;
				else if (key == '/')
					p2.x += 1;
				else if (key == 'l')
					p2.y -= 1;
				else if (key == '.')
					p2.y += 1;
				else if (key == ' ')
				{
					cout << "save this box to #" << flush;
					int num;
					cin >> num;
					if (num < roiBox.size())
						roiBox[num] = cv::Rect(p1 / ratio, p2 / ratio);
					else
						cout << "wrong number!" << endl;
					pushROI = false;
					cv::destroyWindow("ROI - 2X view");
				}
			}
			else if (key == ' ')
				showName = !showName;
			else if (key == '/')
				stop = !stop;
			else if (key == ']')
				proc.SetBinThresh(proc.GetBinThresh() + 1);
			else if (key == '[')
				proc.SetBinThresh(proc.GetBinThresh() - 1);
			else if (key == 'r')
			{
				std::cout << "Reset ROI boxes!" << std::endl;
				roiBox.clear();
			}
			// else if (key == 'x')
			// {
			// 	std::cout << "Erase the last ROI box!" << std::endl;
			// 	roiBox.pop_back();
			// }
			else if (key == 'b')
			{
				cout << "Set background color : " << rgb << endl;
				proc.SetBGcolor(rgb);
			}
			else if (key == 'c')
			{
				adjustHeight = !adjustHeight;
				if (adjustHeight)
					cout << "Confirm the box selection and measure bottom and top of each box. Press 'c' again to stop the measurement" << endl;
				else
					cout << "Adjusted boxes are displayed" << endl;
			}
			else if (key == 'p')
			{
				string name;
				cout << "file suffix: " << flush;
				cin >> name;
				for (int i = 0; i < roiBox.size(); i++)
				{
					if (roiCnt[i] < 10)
						continue;
					int top = floor(roiRange[i].x / (double)roiCnt[i]);
					int bottom = floor(roiRange[i].y / (double)roiCnt[i]) + 1;
					roiBox[i] = cv::Rect(roiBox[i].x, (roiBox[i].y + top), roiBox[i].width, (bottom - top));
				}
				auto config = proc.GetConfig();
				config->setRoiBoxes(roiBox);
				config->saveConfig(dir + "/data/config_" + name + ".yml");
				config->saveBoxConfig(dir + "/data/box_" + name + ".yml");
				cout << "new configuration file was saved as " + dir + "/data/config_" + name + ".yml, box_" + name + ".yml" << endl;
				config->loadConfig("./data/config_" + name + ".yml");
				// config.loadBoxConfig("./data/box1"+name+".yml");
			}
			else if (source == SOURCE::VIDEO)
			{
				if (key == ',')
					cap.set(cv::CAP_PROP_POS_FRAMES, cap.get(cv::CAP_PROP_POS_FRAMES) - 10);
				else if (key == '.')
					cap.set(cv::CAP_PROP_POS_FRAMES, cap.get(cv::CAP_PROP_POS_FRAMES) + 10);
			}
		}
		// char key = cv::waitKey(1);
		if (source == SOURCE::VIDEO && stop)
			cap.set(cv::CAP_PROP_POS_FRAMES, cap.get(cv::CAP_PROP_POS_FRAMES) - 1);

		if (getWindowProperty("Setting", WND_PROP_AUTOSIZE) != 1)
			break;
	}
	// cv::destroyWindow("Setting");
	// firstLoop = false;

	// #ifdef VIDEO
	// if (key == ' ')
	// {
	// 	int newFrame = cap.get(cv::CAP_PROP_POS_FRAMES) + 100;
	// 	if (newFrame > cap.get(cv::CAP_PROP_FRAME_COUNT))
	// 		newFrame -= cap.get(cv::CAP_PROP_FRAME_COUNT);
	// 	cap.set(cv::CAP_PROP_POS_FRAMES, newFrame);
	// }
	// #endif
}

void OcrMain::RenderForLearning()
{
	bool learning(true);
	while (1)
	{
		cv::Mat img;
		cap >> img;
		cv::Mat imgCopy, imgResize;
		img.copyTo(imgCopy);

		std::vector<cv::Mat> digits;
		std::vector<cv::Rect> bBoxAll;
		for (size_t i = 0; i < roiBox.size(); i++)
		{
			cv::Mat imgCrop = img(roiBox[i]);
			cv::Mat imgBin;
			std::vector<cv::Rect> bBox;
			int top, bottom;
			auto tmp = proc.Process(imgCrop, bBox, imgBin, top, bottom, true);
			digits.insert(digits.end(), tmp.begin(), tmp.end());

			for (cv::Rect box : bBox)
			{
				box.x += roiBox[i].x;
				box.y += roiBox[i].y;
				bBoxAll.push_back(box);
				cv::rectangle(imgCopy, box, CV_RGB(255, 255, 0), 1);
			}
		}
		if (source == SOURCE::VIDEO)
		{
			cv::putText(imgCopy, "frame#: " + to_string((int)cap.get(cv::CAP_PROP_POS_FRAMES)) + "/" + to_string((int)cap.get(cv::CAP_PROP_FRAME_COUNT)), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(60000), 1);
			if (cap.get(cv::CAP_PROP_POS_FRAMES) == cap.get(cv::CAP_PROP_FRAME_COUNT) - 1)
				cap.set(cv::CAP_PROP_POS_FRAMES, 0);
		}
		double ratio = (double)resize / 100.;
		cv::resize(imgCopy, imgCopy, cv::Size(img.cols * ratio, img.rows * ratio));
		cv::imshow("Training", imgCopy);
		cv::createTrackbar(
			"resize", "Training", &resize, 100, [](int pos, void *data) {}, (void *)&imgCopy);

		char key = cv::waitKey(1);

		if (learning)
			for (int i = 0; i < digits.size(); i++)
			{
				cv::rectangle(imgCopy, bBoxAll[i], CV_RGB(255, 0, 0), 1);
				learning = ocr.RecogAndLearn(digits[i]);
				if (!learning)
					break;
			}
		// firstLoop = false;

		if (key == 'p')
		{
			string name;
			cout << "file suffix: " << flush;
			cin >> name;
			ocr.saveTrainingData(dir + "/data/training_" + name + ".yml");
			learning = true;
		}
		else if (key == 's')
		{
			ocr.printTrainingStatus();
			cv::waitKey(0);
			learning = true;
		}
		else if (source == SOURCE::VIDEO && key == ' ')
		{
			int newFrame = cap.get(cv::CAP_PROP_POS_FRAMES) + 100;
			if (newFrame > cap.get(cv::CAP_PROP_FRAME_COUNT))
				newFrame -= cap.get(cv::CAP_PROP_FRAME_COUNT);
			cap.set(cv::CAP_PROP_POS_FRAMES, newFrame);
		}
		if (getWindowProperty("Training", WND_PROP_AUTOSIZE) != 1)
			break;
	}
}
/*
int main(int argc, char **argv)
{

	while (1)
	{
		cv::Mat img;
		cap >> img;
		cv::Mat imgCopy, imgResize;
		img.copyTo(imgCopy);

		std::vector<cv::Mat> digits;
		std::vector<cv::Rect> bBoxAll;
		std::vector<std::string> results;
		bool fluoChk(false);
		for (size_t i = 0; i < roiBox.size(); i++)
		{
			cv::Mat imgCrop = img(roiBox[i]);
			cv::Mat imgBin;
			std::vector<cv::Rect> bBox;
			int top, bottom;
			std::vector<cv::Mat> digits = proc.Process(imgCrop, bBox, imgBin, top, bottom, true);
			string result("");
			bool success = ocr.recognize(digits, result);

			if(i<4)	results.push_back(result);
			else if(i==roiBox.size()-1){
				if(!result.size()) pack[i]=0;
				else if(fluoChk) pack[i] = 1;
				else pack[i]=2;
			}
			else if(success)
			{
				pack[i] = atof(result.c_str());
				if(i==9) fluoChk = success;
			}

			cv::rectangle(imgCopy, roiBox[i], CV_RGB(255, 255, 0), 1);
			cv::putText(imgCopy, result, roiBox[i].tl(), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(60000));
			// cv::imshow("binary", imgBin);
			// cv::imshow("crop", imgCrop);
			// if (firstLoop)
			// 	cv::waitKey(100);
		}
		//option management
		if(results[0]=="long."||results[2].substr(0,3)=="lat.") //if first two box shows table options
		{
			if(results[1].size()) pack[2] = atof(results[1].c_str());
			if(results[3].size()) pack[3] = atof(results[3].c_str());
		}
		else
		{
			if(results[1].size()) pack[0] = atof(results[1].c_str());
			if(results[3].size()) pack[1] = atof(results[3].c_str());
			if(results[0]=="rao") pack[0] = -pack[0];
			if(results[2]=="cran") pack[0] = -pack[0];
		}
		socket.SendFloatBuffer(pack.data(), 375);

#ifdef VIDEO
		cv::putText(imgCopy, "frame#: " + to_string((int)cap.get(cv::CAP_PROP_POS_FRAMES)) + "/" + to_string((int)cap.get(cv::CAP_PROP_FRAME_COUNT)), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(60000), 1);
		if (cap.get(cv::CAP_PROP_POS_FRAMES) == cap.get(cv::CAP_PROP_FRAME_COUNT) - 1)
			cap.set(cv::CAP_PROP_POS_FRAMES, 0);
#endif
		cv::resize(imgCopy, imgResize, cv::Size(floor(img.cols * 0.5), floor(img.rows * 0.5)));
		cv::imshow("CaptureBoard", imgResize(cv::Rect(cv::Point2i(100, imgResize.rows*0.85), cv::Point2i(imgResize.cols-50, imgResize.rows))));
		char key = cv::waitKey(1);

		// firstLoop = false;


#ifdef VIDEO
		// if (key == ' ')
		// {
		// 	int newFrame = cap.get(cv::CAP_PROP_POS_FRAMES) + 100;
		// 	if (newFrame > cap.get(cv::CAP_PROP_FRAME_COUNT))
		// 		newFrame -= cap.get(cv::CAP_PROP_FRAME_COUNT);
		// 	cap.set(cv::CAP_PROP_POS_FRAMES, newFrame);
		// }
#endif
	}

	return 0;
}
*/