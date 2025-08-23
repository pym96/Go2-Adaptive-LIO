
#ifndef CHEETAH_SDUOG_48_VMWBC_CHANGE_F_RL_V1_0_HISTORY_WRAPPER_H
#define CHEETAH_SDUOG_48_VMWBC_CHANGE_F_RL_V1_0_HISTORY_WRAPPER_H
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <cstdlib>
#include "iostream"
#include <fstream>
#include <cmath>
/*
 * Dynamic storage num_history step obs in obs_history
 * Just simple call "add_obs" and "get_obs_history"
 */

namespace wrapper{

    template<typename Dtype, int depth>
    class History_wrapper{
        public:
            History_wrapper(int obs_length, int num_history);
            void add_obs(Eigen::Matrix<Dtype, -1, depth> obs);
            Eigen::Matrix<Dtype, -1, depth> get_obs();
            Eigen::Matrix<Dtype, -1, depth> get_obs_history();
            void reset();
            void set_length(int obs_length, int num_history);
            void init_buffer();

        private:
            Eigen::Matrix<Dtype, -1, depth> _obs;
            Eigen::Matrix<Dtype, -1, depth> _obs_history;
            int _obs_length;
            int _num_history;
            int _obs_history_length;
    };


}

#endif //CHEETAH_SDUOG_48_VMWBC_CHANGE_F_RL_V1_0_HISTORY_WRAPPER_H
