#ifndef TKMETRICSUTILS_H
#define TKMETRICSUTILS_H

#include <iostream>
#include <vector>
#include <sys/types.h>
#include <dirent.h>

#define FatalError(s) {                                                \
    std::stringstream _where, _message;                                \
    _where << __FILE__ << ':' << __LINE__;                             \
    _message << std::string(s) + "\n" << __FILE__ << ':' << __LINE__;\
    std::cerr << _message.str() << "\nAborting...\n";                  \
    exit(EXIT_FAILURE);                                                \
}


std::vector<std::string> getDirectoryFiles(const std::string& name){
    std::vector<std::string> v;
    DIR* dirp = opendir(name.c_str());
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL) {
        std::string filename = dp->d_name;
        if(filename != "." && filename != "..")
            v.push_back(filename);
    }
    closedir(dirp);
    return v;
}

void convertFilename(std::string &filename,const std::string& l_folder, const std::string& i_folder, const std::string& l_ext,const std::string& i_ext){
    filename.replace(filename.find(l_folder),l_folder.length(),i_folder);
    filename.replace(filename.find(l_ext),l_ext.length(),i_ext);
}

bool fileExist(const char *fname) {
    std::ifstream dataFile (fname, std::ios::in | std::ios::binary);
    if(!dataFile)
        return false;
    return true;
}

#endif /*TKMETRICSUTILS_H*/