
#include "history_wrapper.h"
namespace wrapper
{
    template<typename Dtype, int depth>
    History_wrapper<Dtype, depth>::History_wrapper(int obs_length, int num_history) {
        _obs_length = obs_length;
        _num_history = num_history;
        _obs_history_length = _num_history * _obs_length;
        init_buffer();
        std::cout << std::endl;
        std::cout << "[History wrapper] : " << "obs sizes = " << _obs.size() << std::endl;
        std::cout << "[History wrapper] : " << "obs history sizes = " << _obs_history.size() << std::endl;
        std::cout << "[History wrapper] : " << "Matrix depth = " << depth << std::endl;
        std::cout << "-------------------------------------------" << std::endl;
        std::cout << std::endl;
    }

    template<typename Dtype, int depth>
    void History_wrapper<Dtype, depth>::add_obs(Eigen::Matrix<Dtype, -1, depth> obs){
        _obs = obs;
        // 检查_obs_history是否全为0
        if (_obs_history.isZero()) {
            // 如果_obs_history全为0，用当前的obs填满整个_obs_history
            for (int i = 0; i < _num_history; ++i) {
                _obs_history.block(i * _obs_length, 0, _obs_length, depth) = _obs;
            }
        } else {
            // 如果_obs_history不为0，则按照原来的逻辑进行
            _obs_history.head(_obs_history_length - _obs_length) = 
                _obs_history.tail(_obs_history_length - _obs_length);
            _obs_history.tail(_obs_length) = _obs;
        }
        //std::cout << "History wrapper: " << "_obs_history = " << _obs_history << std::endl;
    }

    template<typename Dtype, int depth>
    Eigen::Matrix<Dtype, -1, depth> History_wrapper<Dtype, depth>::get_obs(){
        return _obs;
    }

    template<typename Dtype, int depth>
    Eigen::Matrix<Dtype, -1, depth> History_wrapper<Dtype, depth>::get_obs_history(){
        return _obs_history;
    }

    template<typename Dtype, int depth>
    void History_wrapper<Dtype, depth>::reset(){
        _obs.setZero();
        _obs_history.setZero();
        _obs_length = 0;
        _num_history = 0;
        _obs_history_length = 0;
    }

    template<typename Dtype, int depth>
    void History_wrapper<Dtype, depth>::set_length(int obs_length, int num_history){
        _obs_length = obs_length;
        _num_history = num_history;
        _obs_history_length = _obs_length * _num_history;
    }

    template<typename Dtype, int depth>
    void History_wrapper<Dtype, depth>::init_buffer(){
        _obs.resize(_obs_length, depth);
        _obs.setZero();
        _obs_history.resize(_obs_history_length, depth);
        _obs_history.setZero();
    }

    template class History_wrapper<float, 1>;
    // template class History_wrapper<double, 1>;
}
