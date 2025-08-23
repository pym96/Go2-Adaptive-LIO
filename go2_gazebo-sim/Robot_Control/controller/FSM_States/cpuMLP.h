//
// Created by billchent on 2022/3/22.
//

#ifndef PROJECTNAME_CPUMLP_H
#define PROJECTNAME_CPUMLP_H

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <cstdlib>
#include "iostream"
#include <fstream>
#include <cmath>

namespace rai {

namespace FuncApprox {

enum class ActivationType {
  linear,
  relu,
  leakyrelu,
  tanh,
  softsign,
  elu
};

template<typename Dtype, ActivationType activationType>
struct Activation {
  inline void nonlinearity(Eigen::Matrix<Dtype, -1, 1> &output) {} 
};

template<typename Dtype>
struct Activation<Dtype, ActivationType::elu> {
    inline void nonlinearity(Eigen::Matrix<Dtype, -1, 1> &output) {
        float alpha = 1.0;
        for (int i = 0; i < output.size(); i++) {
            if (output[i]<0)
                output[i] = alpha * (std::exp(output[i])-1);
            else
                output[i] = output[i];

        }
    }
};

template<typename Dtype>
struct Activation<Dtype, ActivationType::relu> {
  inline void nonlinearity(Eigen::Matrix<Dtype, -1, 1> &output) {
    output = output.cwiseMax(0.0);
  }
};

template<typename Dtype>
struct Activation<Dtype, ActivationType::leakyrelu> {
  inline void nonlinearity(Eigen::Matrix<Dtype, -1, 1> &output) {
    output = output.cwiseMax(1e-2*output);
  }
};

template<typename Dtype>
struct Activation<Dtype, ActivationType::tanh> {
  inline void nonlinearity(Eigen::Matrix<Dtype, -1, 1> &output) {
    output = output.array().tanh();
  }
};

template<typename Dtype>
struct Activation<Dtype, ActivationType::softsign> {
  inline void nonlinearity(Eigen::Matrix<Dtype, -1, 1> &output) {
    for (int i = 0; i < output.size(); i++) {
      output[i] = output[i] / (std::abs(output[i]) + 1.0);
    }
  }
};

template<typename Dtype, int StateDim, int ActionDim, ActivationType activationType>
class MLP_fullyconnected { 

 public:
  typedef Eigen::Matrix<Dtype, ActionDim, 1> Action;
  typedef Eigen::Matrix<Dtype, StateDim, 1> State;

  MLP_fullyconnected(std::vector<int> hiddensizes) {
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

    layersizes.push_back(StateDim); 
    layersizes.reserve(layersizes.size() + hiddensizes.size());
    layersizes.insert(layersizes.end(), hiddensizes.begin(), hiddensizes.end());
    layersizes.push_back(ActionDim); 

    params.resize(2 * (layersizes.size() - 1)); 
    Ws.resize(layersizes.size() - 1); 
    bs.resize(layersizes.size() - 1); 
    lo.resize(layersizes.size()); 
    Stdev.resize(ActionDim); 

    for (int i = 0; i < (int)(params.size()); i++) { 
      if (i % 2 == 0) 
      {
        Ws[i / 2].resize(layersizes[i / 2 + 1], layersizes[i / 2]);
        params[i].resize(layersizes[i / 2] * layersizes[i / 2 + 1]); 
      }
      if (i % 2 == 1) 
      {
        bs[(i - 1) / 2].resize(layersizes[(i + 1) / 2]); 
        params[i].resize(layersizes[(i + 1) / 2]);
      }
    }
    std::cout<<"layersizes: "<<layersizes.size()<<"\t layers: "<<layersizes[0]<<"\t"<<layersizes[1]<<"\t"<<layersizes[2]<<"\t"<<layersizes[3]<<"\t"<<layersizes[4]<<std::endl;
    std::cout<<"Ws.size"<<Ws.size()<<std::endl;
    std::cout<<"params..size"<<params.size()<<std::endl;
  }

  void updateParamFromTxt(std::string fileName) {
    const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n"); 

    std::ifstream indata;
    indata.open(fileName);
      if(!indata)
      {
          std::cout <<"打开文件失败！!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" <<std::endl;
      }
    std::string line;
    getline(indata, line); //读取一行
    std::stringstream lineStream(line);
    std::string cell;

    int totalN = 0;
    ///assign parameters
    std::cout<<"updateParamFromTxt params.size():"<<params.size()<<std::endl; 
    for (int i = 0; i < (int)(params.size()); i++) { 
      int paramSize = 0;

      while (std::getline(lineStream, cell, ',')) { 
        params[i](paramSize++) = std::stod(cell); 
        if (paramSize == params[i].size()) break; 
      }
      totalN += paramSize;
      if (i % 2 == 0)
        memcpy(Ws[i / 2].data(), params[i].data(), sizeof(Dtype) * Ws[i / 2].size());
      if (i % 2 == 1)
        memcpy(bs[(i - 1) / 2].data(), params[i].data(), sizeof(Dtype) * bs[(i - 1) / 2].size());
    }

    printf("load weight and bias from txt OK!!!!!!!!!!!!!!!!!!!!!!!!\n");
  }

  inline Action forward(State &state) {

    lo[0] = state;

    for (int cnt = 0; cnt < (int)(Ws.size()) - 1; cnt++) { 
      lo[cnt + 1] = Ws[cnt] * lo[cnt] + bs[cnt];
      activation_.nonlinearity(lo[cnt + 1]);
    }
    lo[lo.size() - 1] = Ws[Ws.size() - 1] * lo[lo.size() - 2] + bs[bs.size() - 1]; 
    return lo.back();
  }

 private:
  std::vector<Eigen::Matrix<Dtype, -1, 1>> params;
  std::vector<Eigen::Matrix<Dtype, -1, -1>> Ws;
  std::vector<Eigen::Matrix<Dtype, -1, 1>> bs;
  std::vector<Eigen::Matrix<Dtype, -1, 1>> lo;

  Activation<Dtype, activationType> activation_;
  Action Stdev;

  std::vector<int> layersizes;
  bool isTanh = false;
};

}

}

#endif //PROJECTNAME_CPUMLP_H
