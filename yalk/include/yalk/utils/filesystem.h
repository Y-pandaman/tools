#pragma once

#include <string>

namespace yalk {

/// \brief read data from the given file.
/// \param file The file path.
/// \param data The data read from the file.
/// \return True if the operation is successful, false otherwise.
bool ReadDataFromFile(const std::string& file, std::string& data);

/// \brief write data to the given file.
/// \param file The file path.
/// \param data The data to be saved.
/// \return True if the operation is successful, false otherwise.
bool WriteDataToFile(const std::string& file, const std::string& data);

/// \brief Get a path to the directory where the application data can be stored.
/// \param app_name The name of the application.
/// \return The path to the app data directory.
std::string GetAppDataDirectory(const std::string& app_name);

/// \brief Touch a file (create it if it does not exist).
/// \param file The file path.
void TouchFile(const std::string& file);

}  // namespace yalk
