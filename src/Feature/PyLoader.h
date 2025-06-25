#pragma once
#include <iostream>
#include <thread>
#undef slots
#include <cmath>
#include <Python.h>
#include "../Lib/site-packages/numpy/core/include/numpy/arrayobject.h"
#define slots Q_SLOTS

struct CGlobalFeatureExtractonOptions
{
	std::string basePath = "D:/GWT/On_the_fly_SfME/src/Feature/GlobalFeature"; //权值文件有关存储路径
	std::string pthPath = basePath+"/vgg16-397923af.pth";
	std::string HDF5Path = basePath + "/VGG16_64_desc_cen.hdf5";
	std::string checkPointPath = basePath + "/VGG16_NetVlad_NoSplit.pth.tar";
	std::string PCAModelPath = basePath + "/PCA_dims32768to2048.model";
};


class PythonThreadLocker
{
	PyGILState_STATE state;
public:
	PythonThreadLocker() : state(PyGILState_Ensure()){}
	~PythonThreadLocker() {
		PyGILState_Release(state);
	}
};

class PyLoader final {
public:
	explicit PyLoader();
	void Initialize();
	static PyObject* GetFunc(PyObject* PyInstance , const std::string& func);
	static PyObject* CallFunc(PyObject* funcPtr,PyObject* args);
	static PyObject* GlobalExtractInstance;
	static PyObject* DeepImageMatchingInstance;

	static PyObject* GlobalExtractorFunc;
	static PyObject* LocalExtractorFunc;
	static PyObject* LocalFeatureSaveFunc;
	static PyObject* LocalFeatureReadFunc;
	static PyObject* LocalMatcherFunc;
	static PyObject* SimilarityMatcher;
};