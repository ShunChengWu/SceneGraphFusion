//
// Created by sc on 10/7/20.
//

#ifndef GRAPHSLAM_MEMORYBLOCK_H
#define GRAPHSLAM_MEMORYBLOCK_H
#include <ORUtils/Vector.h>
#include <ORUtils/Logging.h>
#include <cassert>
#include <cstring>

template<typename T>
class PtrStride {
public:
    T *data;
    size_t stride=1;
    PtrStride(T *data, size_t stride): data(data), stride(stride){}
    T& operator [](size_t i) {return data[i*stride];}
};

class MemoryBlock {
public:
    enum DATA_TYPE{
        FLOAT, INT, INT64_T, NONE
    };
    MemoryBlock(){
//        SCLOG(VERBOSE) << this <<  " Constructor called ";
    }

    MemoryBlock(DATA_TYPE dataType, size_t size) {
        Allocate(dataType,size);
    }
    MemoryBlock(void* data, DATA_TYPE dataType, size_t size) {
        mData = data;
        mSize = size;
        mDtype = dataType;
        bNeedDelete = false;
    }
    ~MemoryBlock(){
//        SCLOG(VERBOSE) << this <<  " destructor called ";
        deAllocate();
    }

    MemoryBlock(MemoryBlock&& source){ // move constructor
//        SCLOG(VERBOSE) << this << " move constructor called";
        this->bNeedDelete = source.bNeedDelete;
        this->mData = source.mData;
        this->mSize = source.mSize;
        this->mDtype = source.mDtype;
        source.bNeedDelete=false; // prevent source release the memory
    }

    MemoryBlock(MemoryBlock const &x) = delete;

    void operator=(MemoryBlock const &x){ //  copy assignment operator
        if (!this->mData) SCLOG(ERROR) << "block was not initialized.";
        if (this->mDtype != x.mDtype) SCLOG(ERROR) << "";
        if (this->mSize != x.mSize) SCLOG(ERROR) << "size mismatch";
//        SCLOG(VERBOSE) << "assign copy operator";
        memcpy(this->mData, x.mData, this->bytesize()*x.mSize);
    }

    void operator=(MemoryBlock &&source){ //  move assignment operator
//        SCLOG(VERBOSE) << this << " move assign called";
        this->bNeedDelete = source.bNeedDelete;
        this->mData = source.mData;
        this->mSize = source.mSize;
        this->mDtype = source.mDtype;
        source.bNeedDelete=false; // prevent source release the memory
    }


    void Clear(){
        memset(mData,0,GetSize(mDtype)*mSize);
    }

    void Resize(DATA_TYPE dataType, size_t size) {
        deAllocate();
        Allocate(dataType,size);
    }

    void ConservativeResize(DATA_TYPE dataType, size_t size) {
        if(dataType != mDtype) throw std::runtime_error("type must be the same");
        auto tmp = MemoryBlock(dataType,size);
        memcpy(tmp.data(), this->data(), GetSize(dataType)*size);
        deAllocate();
        Allocate(dataType,size);
        memcpy(this->data(), tmp.data(), GetSize(dataType)*size);
    }

    template<typename T>
    T* Get() const {return (T*)mData;}

    template<typename T>
    T& at(size_t x) const {
        if (x >= mSize) throw std::runtime_error("out of range.\n");
        return Get<T>()[x];
    }

    friend std::ostream& operator<<(std::ostream& os, const MemoryBlock& dt){
        switch (dt.mDtype) {
            case FLOAT:
                print<float>(os,dt);
                break;
            case INT:
                print<int>(os,dt);
                break;
            case INT64_T:
                print<int64_t>(os,dt);
                break;
            case NONE:
                break;
        }
        return os;
    }

    size_t bytesize() const {return GetSize(mDtype);}
    void * data() const {return mData;}
    size_t size() const {return mSize;}
    DATA_TYPE type() const {return mDtype;}
protected:
    void *mData = nullptr;
    DATA_TYPE mDtype=NONE;
    size_t mSize=0;
    bool bNeedDelete=false;
private:
    size_t GetSize(DATA_TYPE dataType) const {
        switch (dataType) {
            case FLOAT:
                return sizeof(float);
            case INT:
                return sizeof(int);
            case NONE:
                throw std::runtime_error("not initialized yet.");
            case INT64_T:
                return sizeof(int64_t);
            default:
                throw std::runtime_error("a new behaviour must be defined after adding new type");
        }
    }
    void Allocate(DATA_TYPE dataType, size_t size, bool force_reallocate = false){
        if (dataType == mDtype && size == mSize && !force_reallocate) return;
        assert( !bNeedDelete);
        mSize = size;
        mDtype = dataType;
        mData =  operator new (size*GetSize(dataType));
        bNeedDelete=true;
    }
    void deAllocate(){
        if(bNeedDelete) {
            switch (mDtype) {
                case FLOAT:
                    delete [](float*)mData;
                    break;
                case INT:
                    delete [](int*)mData;
                    break;
                case NONE:
                    break;
                case INT64_T:
                    delete [](int64_t*)mData;
                    break;
                default:
                    throw std::runtime_error("a new behaviour must be defined after adding new type");
            }
            bNeedDelete=false;
        }
    }

    template<typename T>
    static void print(std::ostream& os, const MemoryBlock& dt){
        for(size_t x=0;x<dt.mSize;++x)
            os << dt.at<T>(x) << " ";
        os << "\n";
    }
};

//template <typename T>
//class Vector{
//public:
//    T* data;
//    size_t
//    Vector(T* data, size_t size):data(data){
//    }
//    void operator = (Vector<T> const &x){
//        if(mSize != x.mSize) SCLOG(ERROR) << "size mismatch";
//        if(mDtype != x.mDtype) SCLOG(ERROR) << "type mismatch";
//        MemoryBlock::operator=(x);
//    }
//};

class MemoryBlock2D: public MemoryBlock {
public:
    using MemoryBlock::at;
    using MemoryBlock::Get;
    ORUtils::Vector2<size_t> mDims;
    MemoryBlock2D() {

    }
    MemoryBlock2D(DATA_TYPE dataType, const ORUtils::Vector2<size_t> &dims): MemoryBlock(dataType, dims.Volume()), mDims(dims) {
    }
    MemoryBlock2D(void* data, DATA_TYPE dataType, const ORUtils::Vector2<size_t> &dims): MemoryBlock(data, dataType, dims.Volume()), mDims(dims) {
    }
    MemoryBlock2D(MemoryBlock *block, const ORUtils::Vector2<size_t> &dims): MemoryBlock(block->data(), block->type(), block->size()), mDims(dims) {
        if ( block->size() != mDims.Volume()) throw std::runtime_error("size mismatch!");
    }

//    MemoryBlock2D(MemoryBlock2D const &x): MemoryBlock(x) { // copy constructor
//        mDims = x.mDims;
//    }
    MemoryBlock2D(MemoryBlock2D &&x):MemoryBlock(std::move(x)) { // move constructor
        mDims = std::move(x.mDims);
    }

    MemoryBlock2D(MemoryBlock2D const &x)=delete ; // copy constructor
    void operator=(MemoryBlock2D const &x){ //  copy assignment operato
        mDims = x.mDims;
        MemoryBlock::operator=(x);
    }
    void operator=(MemoryBlock2D &&x){ //  copy assignment operato
        mDims = x.mDims;
        MemoryBlock::operator=(std::move(x));
    }

    void Resize(DATA_TYPE dataType, const ORUtils::Vector2<size_t> &dims){
        mDims = dims;
        MemoryBlock::Resize(dataType,dims.Volume());
    }

    template<typename T>
    T* Get(size_t x, size_t y) const{
        if (x >= mDims.x) throw std::runtime_error("exceed");
        if (y >= mDims.y) throw std::runtime_error("exceed");
        return MemoryBlock::Get<T>() + x*mDims.y+y;
    }

    template<typename T>
    T* Row(size_t x) const {
        if (x >= mDims.x) throw std::runtime_error("exceed");
        return MemoryBlock::Get<T>() + x*mDims.y;
    }

    template<typename T>
    PtrStride<T> Col(size_t y) {
        if (y>=mDims.y) throw std::runtime_error("exceed");
        return PtrStride<T>(MemoryBlock::Get<T>() + y, mDims.y);
    }

    template<typename T>
    T& at(size_t x, size_t y) const {
        return Get<T>(x,y)[0];
    }


    template<typename T>
    static void print(std::ostream& os, const MemoryBlock2D& dt){
        for(size_t x=0;x<dt.mDims.x;++x){
            for(size_t y=0;y<dt.mDims.y;++y)
                os << dt.at<T>(x,y) << " ";
            os << "\n";
        }
    }
    friend std::ostream& operator<<(std::ostream& os, const MemoryBlock2D& dt){
        switch (dt.mDtype) {
            case FLOAT:
                print<float>(os,dt);
                break;
            case INT:
                print<int>(os,dt);
                break;
            case INT64_T:
                print<int64_t>(os,dt);
                break;
            case NONE:
                break;
        }
        return os;
    }
};

class MemoryBlock3D: public MemoryBlock {
public:
    using MemoryBlock::Get;
    using MemoryBlock::at;
    ORUtils::Vector3<size_t> mDims;

    MemoryBlock3D(){

    }
    MemoryBlock3D(DATA_TYPE dataType, const ORUtils::Vector3<size_t> &dims):
            MemoryBlock(dataType, dims.x*dims.y*dims.z),mDims(dims) {
    }
    MemoryBlock3D(void* data, DATA_TYPE dataType, const ORUtils::Vector3<size_t> &dims):MemoryBlock(data, dataType, dims.Volume()),mDims(dims) {
    }
    MemoryBlock3D(MemoryBlock2D *block, const ORUtils::Vector3<size_t> &dims): MemoryBlock(block->data(), block->type(), block->size()), mDims(dims) {
        if ( block->size() != dims.Volume()) throw std::runtime_error("size mismatch!");
    }
    MemoryBlock3D(MemoryBlock *block, const ORUtils::Vector3<size_t> &dims): MemoryBlock(block->data(), block->type(), block->size()),mDims(dims) {
        if ( block->size() != dims.Volume()) throw std::runtime_error("size mismatch!");
    }
//    MemoryBlock3D(MemoryBlock3D const &x): MemoryBlock(x) { // copy constructor
//        mDims = x.mDims;
//    }
    MemoryBlock3D(MemoryBlock3D const &x)=delete ; // copy constructor

    MemoryBlock3D(MemoryBlock3D &&x):MemoryBlock(std::move(x)) { // move constructor
        mDims = std::move(x.mDims);
    }

    void operator=(MemoryBlock3D const &x){ //  copy assignment operato
        mDims = x.mDims;
        MemoryBlock::operator=(x);
    }

    void operator=(MemoryBlock3D &&x){ //  move assignment operato
        mDims = x.mDims;
        MemoryBlock::operator=(std::move(x));
    }


    void Resize(DATA_TYPE dataType, const ORUtils::Vector3<size_t> &dims){
        mDims = dims;
        MemoryBlock::Resize(dataType,dims.Volume());
    }

    template<typename T>
    T* Get(size_t x, size_t y, size_t z) {
        if (x >= mDims.x) throw std::runtime_error("exceed");
        if (y >= mDims.y) throw std::runtime_error("exceed");
        if (z >= mDims.z) throw std::runtime_error("exceed");
        return MemoryBlock::Get<T>() + (x*mDims.y+y)*mDims.z+z;
    }

    template<typename T>
    T& at(size_t x, size_t y,size_t z) {
        if (x >= mDims.x) throw std::runtime_error("exceed");
        if (y >= mDims.y) throw std::runtime_error("exceed");
        if (z >= mDims.z) throw std::runtime_error("exceed");
        return Get<T>(x,y,z)[0];
    }
};

//void Test_MemoryBlock(){
//    size_t size = 10;
//    MemoryBlock tmp(MemoryBlock::DATA_TYPE::FLOAT, 10);
//    tmp.Clear();
//    for(size_t i=0;i<size;++i)
//        assert(static_cast<float*>(tmp.data())[i]==0);
//    for(size_t i=0;i<size;++i)
//        assert(tmp.Get<float>()[i] ==0 );
//}
//
//void Test_MemoryBlock2D(){
//    size_t C=2,R=5;
//    MemoryBlock2D tmp (MemoryBlock::DATA_TYPE::FLOAT, {R,C});
//    tmp.Clear();
//    for(size_t r=0;r<R;++r){
//        for(size_t c=0;c<C;++c){
//            std::cout << tmp.at<float>(r,c) << " ";
//        }
//        std::cout << "\n";
//    }
//    for(size_t r=0;r<R;++r){
//        for(size_t c=0;c<C;++c) {
//            tmp.Row<float>(r)[c] = r;
//        }
//    }
//    for(size_t r=0;r<R;++r){
//        for(size_t c=0;c<C;++c){
//            std::cout << tmp.at<float>(r,c) << " ";
//        }
//        std::cout << "\n";
//    }
//    tmp.Clear();
//    for(size_t r=0;r<R;++r){
//        for(size_t c=0;c<C;++c) {
//            tmp.Col<float>(c)[r] = c;
//        }
//    }
//    for(size_t r=0;r<R;++r){
//        for(size_t c=0;c<C;++c){
//            std::cout << tmp.at<float>(r,c) << " ";
//        }
//        std::cout << "\n";
//    }
//    SCLOG(VERBOSE) << "\n";
//}


#endif //GRAPHSLAM_MEMORYBLOCK_H
