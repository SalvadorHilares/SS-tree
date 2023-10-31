#include "SStree.h"

bool SsNode::test(bool isRoot) const {
    size_t count = 0;
    if (this->isLeaf()) {
        const SsLeaf* leaf = dynamic_cast<const SsLeaf*>(this);
        count = leaf->points.size();

        // Verificar si los puntos están dentro del radio del nodo
        for (const Point& point : leaf->points) {
            if (distance(this->centroid, point) > this->radius) {
                std::cout << "Point outside node radius detected." << std::endl;
                return false;
            }
        }
    } else {
        const SsInnerNode* inner = dynamic_cast<const SsInnerNode*>(this);
        count = inner->children.size();

        // Verificar si los centroides de los hijos están dentro del radio del nodo padre
        for (const SsNode* child : inner->children) {
            if (distance(this->centroid, child->centroid) > this->radius) {
                std::cout << "Child centroid outside parent radius detected." << std::endl;
                return false;
            }
            // Verificar recursivamente cada hijo
            if (!child->test(false)) {
                return false;
            }
        }
    }

    // Comprobar la validez de la cantidad de hijos/puntos
    if (!isRoot && (count < Settings::m || count > Settings::M)) {
        std::cout << "Invalid number of children/points detected." << std::endl;
        return false;
    }

    // Comprobar punteros de parentezco, salvo para el nodo raíz
    if (!isRoot && !parent) {
        std::cout << "Node without parent detected." << std::endl;
        return false;
    }

    return true;
}

void SsTree::test() const {
    bool result = root->test(true);

    if (root->parent) {
        std::cout << "Root node parent pointer is not null!" << std::endl;
        result = false;
    }

    if (result) {
        std::cout << "SS-Tree is valid!" << std::endl;
    } else {
        std::cout << "SS-Tree has issues!" << std::endl;
    }
}

void SsNode::print(size_t indent) const {
    for (size_t i = 0; i < indent; ++i) {
        std::cout << "  ";
    }

    // Imprime información del nodo.
    std::cout << "Centroid: " << centroid << ", Radius: " << radius;
    if (isLeaf()) {
        const SsLeaf* leaf = dynamic_cast<const SsLeaf*>(this);
        std::cout << ", Points: [ ";
        for (const Point& p : leaf->points) {
            std::cout << p << " ";
        }
        std::cout << "]";
    } else {
        std::cout << std::endl;
        const SsInnerNode* inner = dynamic_cast<const SsInnerNode*>(this);
        for (const SsNode* child : inner->children) {
            child->print(indent + 1); 
        }
    }
    std::cout << std::endl;
}

void SsTree::print() const {
    if (root) {
        root->print();
    } else {
        std::cout << "Empty tree." << std::endl;
    }
}

void SsLeaf::saveToStream(std::ostream &out) const
{
    bool isLeaf = true;
    out.write(reinterpret_cast<const char*>(&isLeaf), sizeof(isLeaf));

    // Guardar centroid y radius
    out.write(reinterpret_cast<const char*>(&centroid), sizeof(centroid));
    out.write(reinterpret_cast<const char*>(&radius), sizeof(radius));

    // Guardar los puntos
    size_t numPoints = points.size();
    out.write(reinterpret_cast<const char*>(&numPoints), sizeof(numPoints));
    for (const auto& point : points) {
        out.write(reinterpret_cast<const char*>(&point), sizeof(point));
    }

    // Guardar las rutas (paths)
    size_t numPaths = paths.size();
    out.write(reinterpret_cast<const char*>(&numPaths), sizeof(numPaths));
    for (const auto& p : paths) {
        size_t pathLength = p.size();
        out.write(reinterpret_cast<const char*>(&pathLength), sizeof(pathLength));
        out.write(p.c_str(), pathLength);
    }
}

void SsLeaf::loadFromStream(std::istream &in) {
    // Leer centroid y radius
    in.read(reinterpret_cast<char*>(&centroid), sizeof(centroid));
    in.read(reinterpret_cast<char*>(&radius), sizeof(radius));

    // Leer puntos
    size_t numPoints;
    in.read(reinterpret_cast<char*>(&numPoints), sizeof(numPoints));
    points.resize(numPoints);
    for (size_t i = 0; i < numPoints; ++i) {
        in.read(reinterpret_cast<char*>(&points[i]), sizeof(points[i]));
    }

    // Leer rutas (paths)
    size_t numPaths;
    in.read(reinterpret_cast<char*>(&numPaths), sizeof(numPaths));
    paths.resize(numPaths);
    for (size_t i = 0; i < numPaths; ++i) {
        size_t pathLength;
        in.read(reinterpret_cast<char*>(&pathLength), sizeof(pathLength));
        char* buffer = new char[pathLength + 1];
        in.read(buffer, pathLength);
        buffer[pathLength] = '\0';
        paths[i] = std::string(buffer);
        delete[] buffer;
    }
}

void SsInnerNode::saveToStream(std::ostream &out) const
{
    bool isLeaf = false;
    out.write(reinterpret_cast<const char*>(&isLeaf), sizeof(isLeaf));

    // Guardar centroid y radius
    out.write(reinterpret_cast<const char*>(&centroid), sizeof(centroid));
    out.write(reinterpret_cast<const char*>(&radius), sizeof(radius));

    // Guardar la cantidad de hijos para saber cuántos nodos leer después
    size_t numChildren = children.size();
    out.write(reinterpret_cast<const char*>(&numChildren), sizeof(numChildren));

    // Guardar los hijos
    for (const auto& child : children) {
        child->saveToStream(out);
    }
}

void SsInnerNode::loadFromStream(std::istream &in) {
    // Leer centroid y radius
    in.read(reinterpret_cast<char*>(&centroid), sizeof(centroid));
    in.read(reinterpret_cast<char*>(&radius), sizeof(radius));

    // Leer cantidad de hijos
    size_t numChildren;
    in.read(reinterpret_cast<char*>(&numChildren), sizeof(numChildren));

    // Leer hijos
    for (size_t i = 0; i < numChildren; ++i) {
        bool childIsLeaf;
        in.read(reinterpret_cast<char*>(&childIsLeaf), sizeof(childIsLeaf));
        
        SsNode* child = childIsLeaf ? static_cast<SsNode*>(new SsLeaf()) : static_cast<SsNode*>(new SsInnerNode());
        child->loadFromStream(in);
        children.push_back(child);
    }
}

/* FUNCIONES DE SSNODE */

size_t SsNode::directionOfMaxVariance() const
{
    NType maxVariance = 0;
    size_t direction = 0;
    std::vector<Point> centroids = this->getEntriesCentroids();
    for (size_t i = 0; i < this->centroid.dim(); i++){
        NType variance = this->varianceAlongDirection(centroids, i);
        if (variance > maxVariance){
            maxVariance = variance;
            direction = i;
        }
    }
    return direction;
}

size_t SsNode::findSplitIndex()
{
    size_t coordinateIndex = this->directionOfMaxVariance();
    this->sortEntriesByCoordinate(coordinateIndex);
    size_t splitIndex = this->minVarianceSplit(coordinateIndex);
    return splitIndex;
}

NType SsNode::varianceAlongDirection(const std::vector<Point> &centroids, size_t direction) const
{
    if (centroids.empty() || centroids.size() == 0)
        return 0;
    NType mean = 0;
    for (Point centroid : centroids)
        mean += centroid[direction];
    mean /= centroids.size();
    NType variance = 0;
    for (Point centroid : centroids)
        variance += (centroid[direction] - mean) * (centroid[direction] - mean);
    return variance;
}

size_t SsNode::minVarianceSplit(size_t coordinateIndex)
{
    NType minVariance = std::numeric_limits<NType>::max();
    size_t splitIndex = Settings::m;
    for(size_t i=Settings::m; i < this->getEntriesCentroids().size() - Settings::m + 1; i++){
        std::vector<Point> left, right;
        for(size_t j=0; j<i; j++)
            left.push_back(this->getEntriesCentroids()[j]);
        for(size_t j=i; j<this->getEntriesCentroids().size(); j++)
            right.push_back(this->getEntriesCentroids()[j]);
        NType variance = this->varianceAlongDirection(left, coordinateIndex) + this->varianceAlongDirection(right, coordinateIndex);
        if (variance < minVariance){
            minVariance = variance;
            splitIndex = i;
        }
    }
    return splitIndex;
}

/* ------------------------------------------------------------- */

/* SSINNER NODE FUNCIONES */

void SsInnerNode::FNDFTrav(const Point &q, size_t k, std::priority_queue<Pair, std::vector<Pair>, Comparator> &L, NType &Dk) const
{
    for (SsNode* child : this->children){
        if (child->intersectsPoint(q)){
            child->FNDFTrav(q, k, L, Dk);
        }
    }
}

std::vector<Point> SsInnerNode::getEntriesCentroids() const
{
    std::vector<Point> centroids;
    for (SsNode* child : this->children)
        centroids.push_back(child->centroid);
    return centroids;
}

void SsInnerNode::sortEntriesByCoordinate(size_t coordinateIndex)
{
    std::sort(this->children.begin(), this->children.end(), [coordinateIndex](SsNode* a, SsNode* b){
        return a->centroid[coordinateIndex] < b->centroid[coordinateIndex];
    });
}

std::pair<SsNode *, SsNode *>* SsInnerNode::split()
{
    size_t splitIndex = this->findSplitIndex();
    SsNode* newNode1 = new SsInnerNode();
    SsNode* newNode2 = new SsInnerNode();
    for(size_t i=0; i<splitIndex; i++){
        dynamic_cast<SsInnerNode*>(newNode1)->children.push_back(this->children[i]);
        this->children[i]->parent = newNode1;  // Assigning parent node
    }
    for(size_t i=splitIndex; i<this->children.size(); i++){
        dynamic_cast<SsInnerNode*>(newNode2)->children.push_back(this->children[i]);
        this->children[i]->parent = newNode2;  // Assigning parent node
    }
    return new std::pair<SsNode*, SsNode*>(newNode1, newNode2);
}

SsNode *SsInnerNode::findClosestChild(const Point &target) const
{
    if (this->isLeaf())
        throw std::runtime_error("findClosestChild called on leaf node");
    NType minDistance = distance(this->children[0]->centroid, target);
    SsNode* result = this->children[0];
    for (SsNode* child : this->children){
        if (distance(child->centroid, target) < minDistance){
            minDistance = distance(child->centroid, target);
            result = child;
        }
    }
    return result;
}

void SsInnerNode::updateBoundingEnvelope()
{
    if (this->children.empty() || this->children.size() == 0)
        return;
    std::vector<Point> points = this->getEntriesCentroids();
    this->centroid = Point(this->children[0]->centroid.dim());
    for(Point point : points)
        for(size_t i=0; i<this->centroid.dim(); i++)
            this->centroid[i] += point[i];
    for(size_t i=0; i<this->centroid.dim(); i++)
        this->centroid[i] /= points.size();
    this->radius = 0;
    for(Point point : points)
        this->radius = std::max(this->radius, distance(this->centroid, point) + point.norm());
}

std::pair<SsNode*, SsNode*> *SsInnerNode::insert(const Point &point)
{
    if (this->isLeaf())
        throw std::runtime_error("insert called on leaf node");
    SsNode* closestChild = this->findClosestChild(point);    
    std::pair<SsNode*, SsNode*>* newNodes = closestChild->insert(point);
    if (&newNodes->first == nullptr){
        this->updateBoundingEnvelope();
        return nullptr;
    }
    else{
        this->children.erase(std::remove(this->children.begin(), this->children.end(), closestChild), this->children.end());
        newNodes->first->updateBoundingEnvelope();
        newNodes->second->updateBoundingEnvelope();
        this->children.push_back(newNodes->first);
        this->children.push_back(newNodes->second);
        this->updateBoundingEnvelope();
        newNodes->first->parent = this;
        newNodes->second->parent = this;
        if (this->children.size() <= Settings::M)
            return nullptr;
    }
    return this->split();
}

/* ------------------------------------------------------------- */


/* ------------------------------------------------------------- */

/* SSLEAF FUNCIONES */

void SsLeaf::FNDFTrav(const Point &q, size_t k, std::priority_queue<Pair, std::vector<Pair>, Comparator> &L, NType &Dk) const
{
    for (Point point : this->points){
        NType dist = distance(point, q);
        if (L.size() < k){
            L.push(Pair(point, dist));
            Dk = std::max(Dk, dist);
        }
        else if (dist < Dk){
            L.pop();
            L.push(Pair(point, dist));
            Dk = L.top().distance;
        }
    }
}

std::vector<Point> SsLeaf::getEntriesCentroids() const
{
    return this->points;
}

void SsLeaf::sortEntriesByCoordinate(size_t coordinateIndex)
{
    std::sort(this->points.begin(), this->points.end(), [coordinateIndex](Point a, Point b){
        return a[coordinateIndex] < b[coordinateIndex];
    });
}

std::pair<SsNode *, SsNode *>* SsLeaf::split()
{
    size_t splitIndex = this->findSplitIndex();
    SsNode* newNode1 = new SsLeaf();
    SsNode* newNode2 = new SsLeaf();
    for(size_t i=0; i<splitIndex; i++)
        dynamic_cast<SsLeaf*>(newNode1)->points.push_back(this->points[i]);
    for(size_t i=splitIndex; i<this->points.size(); i++)
        dynamic_cast<SsLeaf*>(newNode2)->points.push_back(this->points[i]);
    return new std::pair<SsNode*, SsNode*>(newNode1, newNode2);
}

void SsLeaf::updateBoundingEnvelope()
{
    if (this->points.empty() || this->points.size() == 0)
        return;
    this->centroid = Point(this->points[0].dim());
    for(Point point : points)
        for(size_t i=0; i<this->centroid.dim(); i++)
            this->centroid[i] += point[i];
    for(size_t i=0; i<this->centroid.dim(); i++)
        this->centroid[i] /= points.size();
    this->radius = 0;
    for(Point point : points)
        this->radius = std::max(this->radius, distance(this->centroid, point) + point.norm());
}

std::pair<SsNode*, SsNode*> *SsLeaf::insert(const Point &point)
{
    if (std::find(points.begin(), points.end(), point) != points.end())
        return nullptr;
    this->points.push_back(point);
    this->updateBoundingEnvelope();
    if (this->points.size() <= Settings::M)
        return nullptr;
    return this->split();
}

/* SSTREE FUNCION */

/* ------------------------------------------------------------- */

SsNode *SsTree::search(SsNode *node, const Point &target)
{
    if (node->isLeaf()){
        SsLeaf* leaf = dynamic_cast<SsLeaf*>(node);
        for (Point point : leaf->points){
            if (point == target){
                return leaf;
            }
        }
    }else{
        SsInnerNode* inner = dynamic_cast<SsInnerNode*>(node);
        for (SsNode* child : inner->children){
            if (child->intersectsPoint(target)){
                SsNode* result = search(child, target);
                if (result != nullptr)
                    return result;
            }
        }
    }
    return nullptr;
}

SsNode *SsTree::searchParentLeaf(SsNode *node, const Point &target)
{
    if (node->isLeaf()){
        return node;
    }else{
        SsInnerNode* inner = dynamic_cast<SsInnerNode*>(node);
        SsNode* child = inner->findClosestChild(target);
        return searchParentLeaf(child, target);
    }
}

void SsTree::insert(const Point &point)
{
    if (root == nullptr){
        root = new SsLeaf();
        dynamic_cast<SsLeaf*>(root)->points.push_back(point);
        this->root->updateBoundingEnvelope();
    }else{
        std::pair<SsNode*, SsNode*>* newNodes = root->insert(point);
        if (&newNodes->first != nullptr){
            SsInnerNode* newRoot = new SsInnerNode();
            newNodes->first->updateBoundingEnvelope();
            newNodes->second->updateBoundingEnvelope();
            newRoot->children.push_back(newNodes->first);
            newRoot->children.push_back(newNodes->second);
            newNodes->first->parent = newRoot;
            newNodes->second->parent = newRoot;
            this->root = newRoot;
        }
    }
}

void SsTree::insert(const Point &point, const std::string &path) 
{
    // Insert the point into the tree using the existing insert function
    insert(point);
    
    // After insertion, search for the leaf node where the point was inserted
    SsNode* targetNode = search(root, point);
    if (targetNode && targetNode->isLeaf()) {
        // Cast the node to a leaf node to access its members
        SsLeaf* targetLeaf = dynamic_cast<SsLeaf*>(targetNode);
        
        // Ensure that the dynamic_cast was successful
        if (targetLeaf) {
            // Check if the point was correctly inserted in the leaf node
            if (std::find(targetLeaf->points.begin(), targetLeaf->points.end(), point) != targetLeaf->points.end()) {
                // Add the path to the paths list of the leaf node
                targetLeaf->getPaths().push_back(path);
            }
        }
    }
}


void SsTree::build(const std::vector<Point> &points)
{
    for (Point point : points)
        this->insert(point);
}

std::vector<std::string> SsTree::kNNQuery(const Point &center, size_t k) const
{
    std::priority_queue<Pair, std::vector<Pair>, Comparator> L;
    NType Dk = std::numeric_limits<NType>::max();
    this->root->FNDFTrav(center, k, L, Dk);
    std::vector<std::string> result;
    while (!L.empty()){
        std::stringstream ss;
        ss << L.top().point;
        result.push_back(ss.str());
        L.pop();
    }
    return result;
}

void SsTree::saveToFile(const std::string &filename) const {
    std::ofstream out(filename, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Cannot open file for writing");
    }
    root->saveToStream(out);
    out.close();
}

void SsTree::loadFromFile(const std::string &filename) {
    std::ifstream in(filename, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Cannot open file for reading");
    }
    if (root) {
        delete root;
        root = nullptr;
    }
    // Aquí se asume que el primer byte determina si es un nodo interno o una hoja
    bool isLeaf;
    in.read(reinterpret_cast<char*>(&isLeaf), sizeof(isLeaf));
    if (isLeaf) {
        root = new SsLeaf();
    } else {
        root = new SsInnerNode();
    }
    root->loadFromStream(in);
    in.close();
}