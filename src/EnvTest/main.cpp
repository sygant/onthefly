//相似纹理测试
int main()
{
	return 0;
}


//#include "PyLoader.h"
//#include "../Base/timer.h"
//#include "../Base/types.h"
//using namespace std;
//
////时间效率测试
////PyObject* Extract(
////    const std::string& imagePath,
////    FeatureKeypoints& keypoints,
////    FeatureDescriptors& descriptors,PyLoader* pyloader)
////{
////    //////////  Superpoint    //////////
////    cout<<"superpoint extract image: "<< imagePath <<endl;
////    PyObject* Arg = PyTuple_Pack(1, PyUnicode_FromString(imagePath.c_str()));
////    PyObject* featureDictPtr = PyObject_CallObject(pyloader->LocalExtractorFunc, Arg);
////    Py_XDECREF(Arg);
////    if (featureDictPtr)
////    {
////        PyObject* keypointsPyPtr = PyDict_GetItemString(featureDictPtr, "keypoints");
////        PyObject* descriptorsPyPtr = PyDict_GetItemString(featureDictPtr, "descriptors");
////        if (keypointsPyPtr)
////        {
////            PyArrayObject* npArray = reinterpret_cast<PyArrayObject*>(keypointsPyPtr);
////            npy_intp* shape = PyArray_SHAPE(npArray);
////            const size_t num_features = shape[0];
////            keypoints.resize(num_features);
////            //cout << "features number: " << num_features << endl;
////            for (npy_intp i = 0; i < shape[0]; ++i)
////            {
////                float x_value = *reinterpret_cast<float*>(PyArray_GETPTR2(npArray, i, 0));
////                float y_value = *reinterpret_cast<float*>(PyArray_GETPTR2(npArray, i, 1));
////                keypoints[i] = FeatureKeypoint(x_value, y_value);
////            }
////        }
////        if (descriptorsPyPtr)
////        {
////            PyArrayObject* npArray = reinterpret_cast<PyArrayObject*>(descriptorsPyPtr);
////            npy_intp* shape = PyArray_SHAPE(npArray);
////            const size_t feature_len = shape[0];
////            const size_t num_features = shape[1];
////            descriptors.resize(num_features, feature_len);
////            for (npy_intp i = 0; i < shape[1]; ++i)
////            {
////                for (npy_intp j = 0; j < shape[0]; ++j)
////                {
////                    float value = *reinterpret_cast<float*>(PyArray_GETPTR2(npArray, j, i));
////                    descriptors(i, j) = value;
////                }
////            }
////
////        }
////    }
////    else
////    {
////        cout << "features null\n";
////        return nullptr;
////    }
////    return featureDictPtr;
////}
//bool GlobalExtract(const std::string& imagePath, GlobalFeature& globalFeature, PyLoader* pyloader)
//{
//    //cout << "start extract global features: "<<imagePath<<endl;
//    PyObject* Arg = PyTuple_Pack(1, PyUnicode_FromString(imagePath.c_str()));
//    PyObject* globalFeatureResultPtr = PyObject_CallObject(pyloader->GlobalExtractorFunc, Arg);
//    Py_DECREF(Arg);
//    if (globalFeatureResultPtr)
//    {
//        PyArrayObject* array = reinterpret_cast<PyArrayObject*>(globalFeatureResultPtr);
//        double* data = reinterpret_cast<double*>(PyArray_DATA(array));
//        globalFeature = GlobalFeature(2048);
//        for (int i = 0; i < 2048; i++)
//        {
//            globalFeature[i] = static_cast<float>(data[i]);
//        }
//        Py_DECREF(globalFeatureResultPtr);
//        return true;
//    }
//    cout << "Unable to get return value" << endl;
//    PyErr_Print();
//    return false;
//}
//
////bool GlobalExtractOri(const std::string& imagePath, GlobalFeature& globalFeature, PyLoader* pyloader)
////{
////    PyObject* Arg = PyTuple_New(1);
////    PyTuple_SetItem(Arg, 0, Py_BuildValue("s", imagePath.c_str()));
////    PyObject* globalFeatureResultPtr = PyObject_CallObject(pyloader->GlobalExtractorFunc, Arg);
////    Py_DECREF(Arg);
////
////    if (globalFeatureResultPtr)
////    {
////        PyArrayObject* array = reinterpret_cast<PyArrayObject*>(globalFeatureResultPtr);
////        double* data = reinterpret_cast<double*>(PyArray_DATA(array));
////        globalFeature = GlobalFeature(2048);
////        for (int i = 0; i < 2048; i++)
////        {
////            globalFeature[i] = static_cast<float>(data[i]);
////        }
////        Py_DECREF(globalFeatureResultPtr);
////        return true;
////    }
////    cout << "Unable to get return value" << endl;
////    PyErr_Print();
////    return false;
////}
//
//int main()
//{
//	PyLoader* pyloader = new PyLoader();
//	
//    //superpoint提取平均40ms
//    /*for (size_t i = 0; i < 10; i++)
//    {
//        Timer ExtractionTimer;
//        ExtractionTimer.Start();
//
//        FeatureKeypoints keypoints1;
//        FeatureDescriptors descriptors1;
//        PyObject* featdict1 = Extract(R"(D:\Dataset\FBK\FBK_collaborative_smartphone\CaffeItalia\acquisitions\merge\11_00004037.jpg)", keypoints1, descriptors1);
//
//        const size_t ExtractionTime = ExtractionTimer.ElapsedMicroSeconds() / 1000;
//        cout << "extract time: " << ExtractionTime << endl;
//    }*/
//
//    //global提取平均800ms
//    for (size_t i = 0; i < 10; i++)
//    {
//        Timer ExtractionTimer;
//        ExtractionTimer.Start();
//
//        GlobalFeature globalFeature;
//        bool flag = GlobalExtract(R"(D:\Dataset\FBK\FBK_collaborative_smartphone\CaffeItalia\acquisitions\merge\11_00004037.jpg)", globalFeature,pyloader);
//
//        const size_t ExtractionTime = ExtractionTimer.ElapsedMicroSeconds() / 1000;
//        cout << "extract time: " << ExtractionTime << endl;
//    }
//    //global原方法提取
//    /*for (size_t i = 0; i < 10; i++)
//    {
//        Timer ExtractionTimer;
//        ExtractionTimer.Start();
//
//        GlobalFeature globalFeature;
//        PyObject* Arg = PyTuple_New(1);
//        PyTuple_SetItem(Arg, 0, Py_BuildValue("s", R"(D:\Dataset\FBK\FBK_collaborative_smartphone\CaffeItalia\acquisitions\merge\11_00004037.jpg)"));
//        PyObject* globalFeatureResultPtr = PyObject_CallObject(PyLoader::GlobalExtractorFunc, Arg);
//        Py_DECREF(Arg);
//
//        if (globalFeatureResultPtr)
//        {
//            const size_t ExtractionTime = ExtractionTimer.ElapsedMicroSeconds() / 1000;
//            cout << "extract time: " << ExtractionTime << endl;
//        }
//    }*/
//	
//	
//
//	return 0;
//}


//Match函数测试
//PyObject* matchFunc = nullptr;
//PyObject* readFunc = nullptr;
//FeatureMatches Match() {
//	PyObject* Arg1 = PyTuple_Pack(1 , PyUnicode_FromString(to_string(1).c_str()));
//	PyObject* feat1 = PyObject_CallObject(readFunc , Arg1);
//	PyObject* Arg2 = PyTuple_Pack(1 , PyUnicode_FromString(to_string(2).c_str()));
//	PyObject* feat2 = PyObject_CallObject(readFunc , Arg2);
//	PyObject* Arg3 = PyTuple_Pack(2 , feat1 , feat2);
//	PyObject* matchesResultPtr = PyObject_CallObject(matchFunc , Arg3);
//
//	FeatureMatches matches;
//	if (matchesResultPtr) {
//		PyArrayObject* npArray = reinterpret_cast<PyArrayObject*>( matchesResultPtr );
//		npy_intp* shape = PyArray_SHAPE(npArray);
//		const size_t num_matches = shape[0];
//		cout << "match numbers: " << num_matches << endl;
//		matches.resize(num_matches);
//		for (npy_intp i = 0; i < shape[0]; ++i) {
//			int x_value = *reinterpret_cast<int*>( PyArray_GETPTR2(npArray , i , 0) );
//			int y_value = *reinterpret_cast<int*>( PyArray_GETPTR2(npArray , i , 1) );
//			matches[i] = FeatureMatch(x_value , y_value);
//		}
//		Py_XDECREF(npArray);
//	}
//	Py_XDECREF(Arg1); Py_XDECREF(Arg2); Py_XDECREF(Arg3);
//	Py_XDECREF(feat1); Py_XDECREF(feat2);
//	return matches;
//}
//void runFunc() {
//	PyLoader* pyloader = new PyLoader();
//	matchFunc = pyloader->GetFunc(PyLoader::DeepImageMatchingInstance , "match");
//	readFunc = pyloader->GetFunc(PyLoader::DeepImageMatchingInstance , "read_features");
//	while (true)
//	{
//		this_thread::sleep_for(chrono::milliseconds(500));
//		auto matches = Match();
//	}
//}
//int main() {
//	thread runThread = thread(runFunc);
//    runThread.detach();
//
//    while (true)
//    {
//        this_thread::sleep_for(chrono::milliseconds(5000));
//    }
//	return 0;
//}



//多线程测试
//PyLoader* pyloader = nullptr;
//PyObject* extractFunc = nullptr;
//PyObject* saveFunc = nullptr;
//CPythonExtractor* pyextractor;
//CGlobalFeatureExtractor* globalextractor;
//
//void test01() {
//    PyLoader* pyloader = new PyLoader();
//    PyObject* extractFunc = pyloader->GetFunc(PyLoader::DeepImageMatchingInstance , "extract");
//    PyObject* saveFunc = pyloader->GetFunc(PyLoader::DeepImageMatchingInstance , "save_features");
//    PyObject* Arg1 = PyTuple_Pack(1 , PyUnicode_FromString(R"(D:\CS\study\Lightglue\pyscript\images\00001.jpg)"));
//    PyObject* features = PyObject_CallObject(extractFunc , Arg1);
//    if (PyDict_Check(features)) {
//        PyObject* keypointsPyPtr = PyDict_GetItemString(features , "keypoints");
//        PyObject* descriptorsPyPtr = PyDict_GetItemString(features , "descriptors");
//        PyArrayObject* npArray = reinterpret_cast<PyArrayObject*>( keypointsPyPtr );
//        npy_intp* shape = PyArray_SHAPE(npArray);
//        const size_t num_features = shape[0];
//        for (npy_intp i = 0; i < shape[0]; ++i) {
//            float x_value = *reinterpret_cast<float*>( PyArray_GETPTR2(npArray , i , 0) );
//            float y_value = *reinterpret_cast<float*>( PyArray_GETPTR2(npArray , i , 1) );
//            //cout << x_value << " , " << y_value << endl;
//        }
//
//        npArray = reinterpret_cast<PyArrayObject*>( descriptorsPyPtr );
//        shape = PyArray_SHAPE(npArray);
//        const size_t feature_len = shape[0];
//        const size_t num_features2 = shape[1];
//        for (npy_intp i = 0; i < shape[1]; ++i) {
//            for (npy_intp j = 0; j < shape[0]; ++j) {
//                float value = *reinterpret_cast<float*>( PyArray_GETPTR2(npArray , i , j) );
//                //cout << value;
//            }
//            //cout << endl;
//        }
//    }
//    PyObject* Arg2 = PyTuple_Pack(2 , features , PyUnicode_FromString("test"));
//    PyObject_CallObject(saveFunc , Arg2);
//}
//
//void initPython() {
//    cout << "init runFunc\n";
//    pyloader = new PyLoader();
//    pyextractor = new CPythonExtractor();
//    CGlobalFeatureExtractonOptions options;
//    globalextractor = new CGlobalFeatureExtractor(options);
//    cout<<"init thread done\n";
//}
//
//void runFunc() {
//    initPython();
//    cout<<"call runFunc\n";
//    while (true) {
//        this_thread::sleep_for(chrono::milliseconds(500));
//        FeatureKeypoints keypoints;
//        FeatureDescriptors descriptors;
//        PyObject* feat = pyextractor->Extract(R"(D:\CS\study\On_the_fly_SfM\src\UI\images\00001.jpg)",keypoints,descriptors);
//        if (feat) {
//            pyextractor->SaveFeatures("test",feat);
//        }
//        //GlobalFeature globalfeat;
//        //globalextractor->Extract(R"(D:\CS\study\On_the_fly_SfM\src\UI\images\00001.jpg)", globalfeat);
//        cout << "done.\n";
//    }
//}
//
//int main() {
//
//    thread runThread = thread(runFunc);
//    runThread.detach();
//
//    while (true)
//    {
//        this_thread::sleep_for(chrono::milliseconds(5000));
//    }
//	return 0;
//}



//初始化以及函数调用测试
//#include "../Base/types.h"
////#include <Python.h>
////#include "../Lib/site-packages/numpy/core/include/numpy/arrayobject.h"
////#include <iostream>
//#include <fstream>
//#include <filesystem>
//#include <string>
//#include "../Feature/PyLoader.h"
//using namespace std;
//
//void test03() {
//    PyLoader* pyloader = new PyLoader();
//
//}
//
//void Extract(const std::string& imagePath, FeatureKeypoints& keypoints , FeatureDescriptors& descriptors);
//void test02();
//int main() {
//    /*test03();*/
//    /*FeatureKeypoints keypoints;
//    FeatureDescriptors descriptors;
//    Extract(R"(D:\CS\study\Lightglue\pyscript\images\00001.jpg)", keypoints , descriptors);
//    cout<<"done.\nkeypoints number: "<< keypoints.size()<<endl;*/
//    return 0;
//}
//
//// 测试---特征提取函数
//void Extract(const std::string& imagePath , FeatureKeypoints& keypoints , FeatureDescriptors& descriptors) {
//    
//    ////////  初始化python环境   ////////
//    cout << "Initializing PyTorch environment..." << endl;
//    Py_SetPythonHome(L"D:\\ENV\\Python\\");
//    Py_SetPath(L"D:\\ENV\\Python\\Lib\\;"
//        L"D:\\ENV\\Python\\Lib\\site-packages\\;"
//        L"D:\\ENV\\Python\\DLLs\\;"
//        L"D:\\ENV\\Python\\Lib\\;"
//        L"D:\\CS\\study\\Lightglue\\pyscript\\deep-image-matching\\src;" //可编辑版本
//        L"D:\\CS\\study\\On_the_fly_SfM\\src\\Feature\\GlobalFeature\\");
//
//    Py_Initialize();
//    if (!Py_IsInitialized())
//    {
//        cout << "Python environment initialization failed!" << endl;
//        PyErr_Print();
//        return;
//    }
//    else {
//        cout << "Python environment initialize success!" << endl;
//    }
//
//    const char* version = Py_GetVersion();
//    cout << "Python version: " << version << endl;
//
//    //////////  导入模块 获取执行函数    //////////
//    _import_array();
//
//    //////////  Initialize Superpoint Function    //////////
//    PyObject* DIMExtractModule = PyImport_ImportModule("pyScript");
//    if (DIMExtractModule == NULL) {
//        PyErr_Print();
//        cout << "Module pyScript not found.\n" << endl;
//        return;
//    }
//    PyObject* DIMExtractClass = PyObject_GetAttrString(DIMExtractModule , "OTFMatcher");
//    if (DIMExtractClass == NULL) {
//        PyErr_Print();
//        cout << "Class OTFMatcher not found.\n" << endl;
//        return;
//    }
//    PyObject* DIMExtractClassInstantiation = PyObject_CallObject(DIMExtractClass , nullptr);
//    Py_XDECREF(DIMExtractClass);
//    if (DIMExtractClassInstantiation == NULL) {
//        PyErr_Print();
//        cout << "Instance OTFMatcher create failed.\n" << endl;
//        return;
//    }
//    PyObject* extractFunc_SP = PyObject_GetAttrString(DIMExtractClassInstantiation , "extract");
//
//    Py_XDECREF(DIMExtractModule);
//    cout << "The local extractor has been successfully loaded!" << endl;
//    
//
//    ////////    提取特征点    ////////
//
//    PyObject* Arg = PyTuple_Pack(1 , PyUnicode_FromString(imagePath.c_str()));
//    PyObject* featureDictPtr = PyObject_CallObject(extractFunc_SP ,Arg);
//    Py_XDECREF(Arg);
//    if (PyDict_Check(featureDictPtr)) {
//        PyObject* keypointsPyPtr = PyDict_GetItemString(featureDictPtr , "keypoints");
//        PyObject* descriptorsPyPtr = PyDict_GetItemString(featureDictPtr , "descriptors");
//        if (PyArray_Check(keypointsPyPtr)) {
//            PyArrayObject* npArray = reinterpret_cast<PyArrayObject*>( keypointsPyPtr );
//            npy_intp* shape = PyArray_SHAPE(npArray);
//            const size_t num_features = shape[0];
//            keypoints.resize(num_features);
//            for (npy_intp i = 0; i < shape[0]; ++i) {
//                float x_value = *reinterpret_cast<float*>( PyArray_GETPTR2(npArray , i , 0) );
//                float y_value = *reinterpret_cast<float*>( PyArray_GETPTR2(npArray , i , 1) );
//                keypoints[i] = FeatureKeypoint(x_value,y_value);
//            }
//            Py_XDECREF(npArray);
//        }
//        if (PyArray_Check(descriptorsPyPtr)) {
//            PyArrayObject* npArray = reinterpret_cast<PyArrayObject*>( descriptorsPyPtr );
//            npy_intp* shape = PyArray_SHAPE(npArray);
//            const size_t feature_len = shape[0];
//            const size_t num_features = shape[1];
//            descriptors.resize(num_features,feature_len);
//            for (npy_intp i = 0; i < shape[1]; ++i) {
//                for (npy_intp j = 0; j < shape[0]; ++j) {
//                    float value = *reinterpret_cast<float*>( PyArray_GETPTR2(npArray , i , j) );
//                    descriptors(i,j) = value;
//                    cout<<value;
//                }
//                cout<<endl;
//            }
//        }
//        Py_XDECREF(keypointsPyPtr);Py_XDECREF(descriptorsPyPtr);
//    }
//    Py_XDECREF(featureDictPtr);
//    Py_XDECREF(extractFunc_SP);
//    return;
//}
//
//void test() {
//    cout << "Initializing PyTorch environment..." << endl;
//    Py_SetPythonHome(L"D:\\ENV\\Python\\");
//    Py_SetPath(L"D:\\ENV\\Python\\Lib\\;"
//        L"D:\\ENV\\Python\\Lib\\site-packages\\;"
//        L"D:\\ENV\\Python\\DLLs\\;"
//        L"D:\\ENV\\Python\\Lib\\;"
//        L"D:\\CS\\study\\On_the_fly_SfM\\src\\Feature\\GlobalFeature\\");
//
//    Py_Initialize();
//    if (!Py_IsInitialized())
//    {
//        cout << "Python environment initialization failed!" << endl;
//        PyErr_Print();
//        return;
//    }
//    else {
//        cout << "Python environment initialize success!" << endl;
//    }
//
//    // 打印 Python 版本信息
//    const char* version = Py_GetVersion();
//    cout << "Python version: " << version << endl;
//
//    _import_array();
//
//    PyObject* sys_path = PySys_GetObject("path");
//    std::cout << "sys.path in Python:\n";
//    for (Py_ssize_t i = 0; i < PyList_Size(sys_path); ++i) {
//        PyObject* path_entry = PyList_GetItem(sys_path , i);
//        if (path_entry != NULL && PyUnicode_Check(path_entry)) {
//            const char* path_str = PyUnicode_AsUTF8(path_entry);
//            std::cout << path_str << "\n";
//        }
//    }
//
//    ifstream fileStream("D:\\CS\\study\\On_the_fly_SfM\\src\\Feature\\GlobalFeature\\module.txt");
//    string line;
//    getline(fileStream , line);
//    fileStream.close();
//    PyObject* importlib = PyImport_ImportModule(line.c_str());
//    if (importlib == NULL) {
//        PyErr_Print();
//        cout << "Module " << line.c_str() << " not found.\n" << endl;
//        return;
//    }
//}
//
//void test02() {
//    cout << "Initializing PyTorch environment..." << endl;
//    Py_SetPythonHome(L"D:\\ENV\\Python\\");
//    Py_SetPath(L"D:\\ENV\\Python\\Lib\\;"
//        L"D:\\ENV\\Python\\Lib\\site-packages\\;"
//        L"D:\\ENV\\Python\\DLLs\\;"
//        L"D:\\ENV\\Python\\Lib\\;"
//        L"D:\\CS\\study\\Lightglue\\pyscript\\deep-image-matching\\src\\;" //可编辑版本
//        L"D:\\CS\\study\\On_the_fly_SfM\\src\\Feature\\GlobalFeature\\");
//
//    Py_Initialize();
//    if (!Py_IsInitialized())
//    {
//        cout << "Python environment initialization failed!" << endl;
//        PyErr_Print();
//        return;
//    }
//    else {
//        cout << "Python environment initialize success!" << endl;
//    }
//
//    // 打印 Python 版本信息
//    const char* version = Py_GetVersion();
//    cout << "Python version: " << version << endl;
//
//    _import_array();
//
//    PyObject* GlobalExtractorModule = PyImport_ImportModule("GlobalFeatureExtractor");
//    if (GlobalExtractorModule == NULL) {
//        PyErr_Print();
//        cout << "Module GlobalFeatureExtractor not found.\n" << endl;
//        return;
//    }
//
//    PyObject* globalFeatureExtractorClass = PyObject_GetAttrString(GlobalExtractorModule , "GlobalFeatureExtractor");
//    if (!globalFeatureExtractorClass)
//    {
//        cout << "Failed to get GlobalFeatureExtractor class!" << endl;
//        PyErr_Print();
//        return;
//    }
//
//    PyObject* args = PyTuple_New(4);
//    PyTuple_SetItem(args , 0 , Py_BuildValue("s" , "D:/CS/study/On_the_fly_SfM/src/Feature/GlobalFeature/vgg16-397923af.pth"));
//    PyTuple_SetItem(args , 1 , Py_BuildValue("s" , "D:/CS/study/On_the_fly_SfM/src/Feature/GlobalFeature/VGG16_64_desc_cen.hdf5"));
//    PyTuple_SetItem(args , 2 , Py_BuildValue("s" , "D:/CS/study/On_the_fly_SfM/src/Feature/GlobalFeature/VGG16_NetVlad_NoSplit.pth.tar"));
//    PyTuple_SetItem(args , 3 , Py_BuildValue("s" , "D:/CS/study/On_the_fly_SfM/src/Feature/GlobalFeature/PCA_dims32768to2048.model"));
//    PyObject* globalFeatureExtractorClassInstantiation = PyObject_CallObject(globalFeatureExtractorClass , args);
//    Py_DECREF(args); Py_DECREF(globalFeatureExtractorClass);
//    if (!globalFeatureExtractorClassInstantiation)
//    {
//        cout << "Instantiation failed!" << endl;
//        PyErr_Print();
//        return;
//    }
//    PyObject* extractFunc = PyObject_GetAttrString(globalFeatureExtractorClassInstantiation , "Extract");
//    Py_DECREF(globalFeatureExtractorClassInstantiation);
//    if (!extractFunc)
//    {
//        cout << "Get function failed!" << endl;
//        PyErr_Print();
//        return;
//    }
//
//    PyObject* DIMExtractModule = PyImport_ImportModule("pyScript");
//    if (DIMExtractModule == NULL) {
//        PyErr_Print();
//        cout << "Module pyScript not found.\n" << endl;
//        return;
//    }
//    PyObject* DIMExtractClass = PyObject_GetAttrString(DIMExtractModule , "OTFMatcher");
//    if (DIMExtractClass == NULL) {
//        PyErr_Print();
//        cout << "Class OTFMatcher not found.\n" << endl;
//        return;
//    }
//    PyObject* DIMExtractInstance = PyObject_CallObject(DIMExtractClass , nullptr);
//    if (DIMExtractInstance == NULL) {
//        PyErr_Print();
//        cout << "Instance OTFMatcher create failed.\n" << endl;
//        return;
//    }
//    PyObject* extractFunc_SP = PyObject_GetAttrString(DIMExtractInstance , "extract");
//    PyObject* Arg = PyTuple_Pack(1 , PyUnicode_FromString(R"(D:\CS\study\Lightglue\pyscript\images\00001.jpg)"));
//    PyObject* featureDictPtr = PyObject_CallObject(extractFunc_SP , Arg);
//    Py_XDECREF(Arg);
//    if (PyDict_Check(featureDictPtr)) {
//        PyObject* keypointsPyPtr = PyDict_GetItemString(featureDictPtr , "keypoints");
//        PyObject* descriptorsPyPtr = PyDict_GetItemString(featureDictPtr , "descriptors");
//        if (PyArray_Check(keypointsPyPtr)) {
//            PyArrayObject* npArray = reinterpret_cast<PyArrayObject*>( keypointsPyPtr );
//            npy_intp* shape = PyArray_SHAPE(npArray);
//            const size_t num_features = shape[0];
//            for (npy_intp i = 0; i < shape[0]; ++i) {
//                float x_value = *reinterpret_cast<float*>( PyArray_GETPTR2(npArray , i , 0) );
//                float y_value = *reinterpret_cast<float*>( PyArray_GETPTR2(npArray , i , 1) );
//                cout << x_value << "," << y_value << endl;
//            }
//            Py_XDECREF(npArray);
//        }
//        Py_XDECREF(keypointsPyPtr); Py_XDECREF(descriptorsPyPtr);
//    }
//    Py_XDECREF(featureDictPtr);
//    Py_XDECREF(extractFunc_SP);
//}
//
//PyObject* testGetFun() {
//    ////////  初始化python环境   ////////
//    cout << "Initializing PyTorch environment..." << endl;
//    Py_SetPythonHome(L"D:\\ENV\\Python\\");
//    Py_SetPath(L"D:\\ENV\\Python\\Lib\\;"
//        L"D:\\ENV\\Python\\Lib\\site-packages\\;"
//        L"D:\\ENV\\Python\\DLLs\\;"
//        L"D:\\ENV\\Python\\Lib\\;"
//        L"D:\\CS\\study\\Lightglue\\pyscript\\deep-image-matching\\src;" //可编辑版本
//        L"D:\\CS\\study\\On_the_fly_SfM\\src\\Feature\\GlobalFeature\\");
//
//    Py_Initialize();
//    if (!Py_IsInitialized())
//    {
//        cout << "Python environment initialization failed!" << endl;
//        PyErr_Print();
//        return nullptr;
//    }
//    else {
//        cout << "Python environment initialize success!" << endl;
//    }
//
//    const char* version = Py_GetVersion();
//    cout << "Python version: " << version << endl;
//
//    //////////  导入模块 获取执行函数    //////////
//    _import_array();
//
//    //////////  Initialize Superpoint Function    //////////
//    PyObject* DIMExtractModule = PyImport_ImportModule("pyScript");
//    if (DIMExtractModule == NULL) {
//        PyErr_Print();
//        cout << "Module pyScript not found.\n" << endl;
//        return nullptr;
//    }
//    PyObject* DIMExtractClass = PyObject_GetAttrString(DIMExtractModule , "OTFMatcher");
//    if (DIMExtractClass == NULL) {
//        PyErr_Print();
//        cout << "Class OTFMatcher not found.\n" << endl;
//        return nullptr;
//    }
//    PyObject* DIMExtractClassInstantiation = PyObject_CallObject(DIMExtractClass , nullptr);
//    Py_XDECREF(DIMExtractClass);
//    if (DIMExtractClassInstantiation == NULL) {
//        PyErr_Print();
//        cout << "Instance OTFMatcher create failed.\n" << endl;
//        return nullptr;
//    }
//    PyObject* extractFunc_SP = PyObject_GetAttrString(DIMExtractClassInstantiation , "extract");
//
//    Py_XDECREF(DIMExtractModule);
//    cout << "The local extractor has been successfully loaded!" << endl;
//    return extractFunc_SP;
//}