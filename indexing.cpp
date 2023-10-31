#include <iostream>
#include <vector>
#include <random>
#include "Point.h"
#include "SStree.h"
#include <nlohmann/json.hpp>
#include <fstream>

struct ImageData {
    Point embedding;
    std::string path;
};

std::vector<ImageData> readEmbeddingsFromJson(const std::string& FILE_NAME) {
    std::vector<ImageData> data;

    try {
        std::ifstream file(FILE_NAME);
        if (!file.is_open()) {
            throw std::runtime_error("Unable to open JSON file.");
        }

        nlohmann::json jsonData;
        file >> jsonData;

        std::vector<std::vector<double>> features = jsonData["features"].get<std::vector<std::vector<double>>>();
        std::vector<std::string> paths = jsonData["paths"].get<std::vector<std::string>>();

        for (size_t i = 0; i < paths.size(); ++i) {
            Point embedding(features[i].size());
            for (size_t j = 0; j < features[i].size(); ++j) {
                embedding[j] = features[i][j];
            }
            data.push_back({embedding, paths[i]});
        }
        
        file.close();
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    return data;
}

int main() {
    const std::string FILE_NAME("../embedding.json");
    std::vector<ImageData> data = readEmbeddingsFromJson(FILE_NAME);

    SsTree tree;
    for (const ImageData& item : data) {
        item.embedding;
        tree.insert(item.embedding, item.path);
    }

    std::string filename = "embbeding.dat";
    tree.saveToFile(filename);

    return 0;
}
