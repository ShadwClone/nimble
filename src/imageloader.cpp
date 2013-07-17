#include <assert.h>
#include <fstream>

#include "imageloader.h"

using namespace std;

Image::Image(char* ps, int w, int h) : pixels(ps), width(w), height(h) {
	
}

Image::~Image() {
	delete[] pixels;
}

namespace {
	//Converts a four-character array to an integer, using little-endian form
	int toInt(const char* bytes) {
		return (int)(((unsigned char)bytes[3] << 24) |
					 ((unsigned char)bytes[2] << 16) |
					 ((unsigned char)bytes[1] << 8) |
					 (unsigned char)bytes[0]);
	}
	
	//Converts a two-character array to a short, using little-endian form
	short toShort(const char* bytes) {
		return (short)(((unsigned char)bytes[1] << 8) |
					   (unsigned char)bytes[0]);
	}
	
	//Reads the next four bytes as an integer, using little-endian form
	int readInt(ifstream &input) {
		char buffer[4];
		input.read(buffer, 4);
		return toInt(buffer);
	}
	
	//Reads the next two bytes as a short, using little-endian form
	short readShort(ifstream &input) {
		char buffer[2];
		input.read(buffer, 2);
		return toShort(buffer);
	}
	
	//Just like auto_ptr, but for arrays
	template<class T>
	class auto_array {
  private:
		T* array;
		mutable bool isReleased;

  public:
		explicit auto_array(T* array_ = NULL): array(array_), isReleased(false) {}
			
			auto_array(const auto_array<T> &aarray) {
				array = aarray.array;
				isReleased = aarray.isReleased;
				aarray.isReleased = true;
			}
			
			~auto_array() {
				if (!isReleased && array != NULL) {
					delete[] array;
				}
			}
			
			T* get() const {return array;}
			
			T* release() {
				isReleased = true;
				return array;
			}
			
			void reset(T* array_ = NULL) {
				if (!isReleased && array != NULL) {
					delete[] array;
				}
				array = array_;
			}
			
			T& operator*() const {return *array;}
			T* operator->() const {return array;}
			T* operator+(int i) {return array + i;}
			T& operator[](int i) {return array[i];}

      void operator=(const auto_array<T> &aarray) {
				if (!isReleased && array != NULL) {
					delete[] array;
				}
				array = aarray.array;
				isReleased = aarray.isReleased;
				aarray.isReleased = true;
			}
  };
}


Image* loadBMP(const char* filename)
{
	ifstream input;
	input.open(filename, ifstream::binary);
	assert(!input.fail() || !"Could not find file");
	char buffer[2];
	input.read(buffer, 2);
	assert(buffer[0] == 'B' && buffer[1] == 'M' || !"Not a bitmap file");
	input.ignore(8);
	int dataOffset = readInt(input);
	
	int headerSize = readInt(input),
	    width = 0,
	    height = 0;

	//Read the header
	switch(headerSize)
  {
		case 12://OS/2 V1
			width = readShort(input);
			height = readShort(input);
			input.ignore(2);//skip "Planes"
			assert(readShort(input) == 24 || !"Image is not 24 bits per pixel");
			break;

		case 40://V3
			width = readInt(input);
			height = readInt(input);
			input.ignore(2);//skip "Planes"
			assert(readShort(input) == 24 || !"Image is not 24 bits per pixel");
			assert(readShort(input) == 0  || !"Image is compressed");
			break;

		case 64://OS/2 V2
			assert(!"Can't load OS/2 V2 bitmaps");
			break;

		case 108://Windows V4
			assert(!"Can't load Windows V4 bitmaps");
			break;

		case 124://Windows V5
			assert(!"Can't load Windows V5 bitmaps");
			break;

		default:
			assert(!"Unknown bitmap format");
	}

	int bytesPerRow = ((3 * width + 3) / 4) * 4;
	int size = bytesPerRow * height;
	
  //Read the data
	auto_array<char> pixelStorage(new char[size]);
	input.seekg(dataOffset, ios_base::beg);
	input.read(pixelStorage.get(), size);
	
	//Get the data into the right format
	auto_array<char> pixels(new char[width * height * 3]);
  for(int row = 0; row < height; row++)
  for(int col = 0; col < width; col++)
  for(int c = 0; c < 3; c++)
  {
    pixels[3 * (width * row + col) + c] =
    pixelStorage[bytesPerRow * row + 3 * col + (2 - c)];
	}
	
	input.close();
  pixelStorage.release();
	return new Image(pixels.release(), width, height);
}









