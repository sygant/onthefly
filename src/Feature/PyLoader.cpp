#include "PyLoader.h"
using namespace std;

PyObject* PyLoader::DeepImageMatchingInstance = nullptr;
PyObject* PyLoader::GlobalExtractInstance = nullptr;

PyObject* PyLoader::GlobalExtractorFunc = nullptr;
PyObject* PyLoader::LocalExtractorFunc = nullptr;
PyObject* PyLoader::LocalFeatureReadFunc = nullptr;
PyObject* PyLoader::LocalFeatureSaveFunc = nullptr;
PyObject* PyLoader::LocalMatcherFunc = nullptr;
PyObject* PyLoader::SimilarityMatcher = nullptr;

PyLoader::PyLoader()
{
    cout << "Initializing PyTorch environment..." << endl;
    Py_SetPythonHome(LR"(D:\GWT\On_the_fly_SfME\thirdparty\Python)");
    Py_SetPath(LR"(D:\GWT\On_the_fly_SfME\thirdparty\Python\Lib\site-packages;)"
        LR"(D:\GWT\On_the_fly_SfME\thirdparty\Python\DLLs;)"
        LR"(D:\GWT\On_the_fly_SfME\thirdparty\Python\Lib;)"
        LR"(D:\GWT\On_the_fly_SfME\src\Feature\GlobalFeature)"//¹¤×÷Ä¿Â¼
        );

    Py_Initialize();
    if (!Py_IsInitialized())
    {
        cout << "Python environment initialization failed!" << endl;
        PyErr_Print();
        return;
    }
    else {
        cout << "Python environment initialize success!" << endl;
    }

    const char* version = Py_GetVersion();
    cout << "Python version: " << version << endl;

    Initialize();
}

void PyLoader::Initialize()
{
    if (!PyEval_ThreadsInitialized())
    {
        PyEval_InitThreads();
    }

    _import_array();

    PyObject* GlobalExtractorModule = PyImport_ImportModule("GlobalFeatureExtractor");
    if (GlobalExtractorModule == NULL) {
        PyErr_Print();
        cout << "Module GlobalFeatureExtractor not found.\n" << endl;
        return;
    }

    PyObject* globalFeatureExtractorClass = PyObject_GetAttrString(GlobalExtractorModule , "GlobalFeatureExtractor");
    if (!globalFeatureExtractorClass)
    {
        cout << "Failed to get GlobalFeatureExtractor class!" << endl;
        PyErr_Print();
        return;
    }

    PyObject* args = PyTuple_New(4);
    CGlobalFeatureExtractonOptions options = CGlobalFeatureExtractonOptions();
    PyTuple_SetItem(args , 0 , Py_BuildValue("s" , options.pthPath.c_str()));
    PyTuple_SetItem(args , 1 , Py_BuildValue("s" , options.HDF5Path.c_str()));
    PyTuple_SetItem(args , 2 , Py_BuildValue("s" , options.checkPointPath.c_str()));
    PyTuple_SetItem(args , 3 , Py_BuildValue("s" , options.PCAModelPath.c_str()));
    PyObject* globalFeatureExtractorClassInstantiation = PyObject_CallObject(globalFeatureExtractorClass , args);
    Py_DECREF(args); Py_DECREF(globalFeatureExtractorClass);
    if (!globalFeatureExtractorClassInstantiation)
    {
        cout << "Instantiation failed!" << endl;
        PyErr_Print();
        return;
    }
    GlobalExtractInstance = globalFeatureExtractorClassInstantiation;
    GlobalExtractorFunc = PyObject_GetAttrString(globalFeatureExtractorClassInstantiation , "Extract");
    if (!GlobalExtractorFunc)
    {
        cout << "Get function failed!" << endl;
        PyErr_Print();
        return;
    }
    cout << "The global extractor has been successfully loaded!" << endl;


    PyObject* DIMExtractModule = PyImport_ImportModule("DIMScript");
    if (DIMExtractModule == NULL) {
        PyErr_Print();
        cout << "Module DIMScript not found.\n" << endl;
        return;
    }
    PyObject* DIMExtractClass = PyObject_GetAttrString(DIMExtractModule , "OTFMatcher");
    if (DIMExtractClass == NULL) {
        PyErr_Print();
        cout << "Class OTFMatcher not found.\n" << endl;
        return;
    }
    PyObject* DIMExtractClassInstantiation = PyObject_CallObject(DIMExtractClass , nullptr);
    Py_XDECREF(DIMExtractClass);
    if (DIMExtractClassInstantiation == NULL) {
        PyErr_Print();
        cout << "Instance OTFMatcher create failed.\n" << endl;
        return;
    }
    DeepImageMatchingInstance = DIMExtractClassInstantiation;
    LocalExtractorFunc = PyObject_GetAttrString(DIMExtractClassInstantiation , "extract");
    if (!LocalExtractorFunc)
    {
        cout << "Get function failed!" << endl;
        PyErr_Print();
        return;
    }
    LocalFeatureSaveFunc = PyObject_GetAttrString(DIMExtractClassInstantiation , "save_features");
    if (!LocalFeatureSaveFunc)
    {
        cout << "Get function failed!" << endl;
        PyErr_Print();
        return;
    }
    cout << "The local extractor has been successfully loaded!" << endl;

    LocalMatcherFunc = PyObject_GetAttrString(DIMExtractClassInstantiation , "match");
    if (!LocalMatcherFunc)
    {
        cout << "Get function failed!" << endl;
        PyErr_Print();
        return;
    }
    LocalFeatureReadFunc = PyObject_GetAttrString(DIMExtractClassInstantiation , "read_features");
    if (!LocalFeatureReadFunc)
    {
        cout << "Get function failed!" << endl;
        PyErr_Print();
        return;
    }
    
    PyObject* SimilarModule = PyImport_ImportModule("Superpoint");
    SimilarityMatcher = PyObject_GetAttrString(SimilarModule, "GetSimilar");
    if (!SimilarityMatcher)
    {
        cout << "Get function failed!" << endl;
        PyErr_Print();
        return;
    }
    cout << "The similar matcher has been successfully loaded!" << endl;
}

PyObject* PyLoader::GetFunc(PyObject* PyInstance,const std::string& func)
{
    PyObject* funcPtr = PyObject_GetAttrString(PyInstance , func.c_str());
    if (!funcPtr)
    {
        cout << "Get function failed!" << endl;
        PyErr_Print();
        return nullptr;
    }
	return funcPtr;
}

PyObject* PyLoader::CallFunc(PyObject* funcPtr , PyObject* args)
{
    PyObject* result = PyObject_CallObject(funcPtr,args);
    return nullptr;
}
