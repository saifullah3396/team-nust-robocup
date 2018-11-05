/**
 * @file Utils/include/FileUtils.h
 *
 * This file declares the class FileUtils.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#ifndef _FILE_UTILS_H_
#define _FILE_UTILS_H_

#include "boost/filesystem.hpp"
#include <iostream>

namespace Utils
{
  /**
   * @class FileUtils
   * @brief Class that provides functions for file handling utilities
   */
  class FileUtils
  {
		public:
			/*static void bool find_file(
				const path & dir_path,         // in this directory,
				const std::string & file_name, // search for this name,
				path & path_found)             // placing path here if found
			{
				if (!exists(dir_path)) 
					return false;

				directory_iterator end_itr; // default construction yields past-the-end

				for (directory_iterator itr(dir_path); itr != end_itr; ++itr)
				{
					if (is_directory(itr->status()))
					{
						if (find_file(itr->path(), file_name, path_found)) 
							return true;
					}
					else if (itr->leaf() == file_name) // see below
					{
						path_found = itr->path();
						return true;
					}
				}
				return false;
			}*/
			
			static int getFileCnt(const string& dir) {
				boost::filesystem::path dirPath(dir);
				if (!boost::filesystem::exists(dirPath))
					return -1;				
				int cnt = 0;
				boost::filesystem::directory_iterator endItr;
				for (boost::filesystem::directory_iterator itr(dirPath); itr != endItr; ++itr)
					cnt++;
				return cnt;
			}
  };
}
#endif //!_FILE_UTILS_H_
