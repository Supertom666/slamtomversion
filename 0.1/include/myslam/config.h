//
// Created by tom on 15/11/2020.
//

#ifndef CONFIG_H
#define CONFIG_H
#include "myslam/commoninclude.h"
namespace myslam
{
class Config
{
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;


    Config() {}//private constructor makes a singleton

public:
    ~Config(); //close the file when deconstructing

    //set a new config file
    static bool setParameterFile(const std::string& filename );

    //access the parameter values
    template< typename T >// access the parameter values，通过模板函数，获得yaml的配置文件中的任意类型的参数值
    static T get ( const std::string& key )
    {
        return T( Config::config_->file_[key] );
    }//之后可以先创建一个yaml文件，利用setParameterFile读取，然后用get来获得文件中的参数
};
}


#endif //INC_0_1_CONFIG_H
