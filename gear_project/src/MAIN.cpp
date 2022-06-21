#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <vector>
#include <tuple>
#include <memory>

using std::string;
using std::cout;
using std::cerr;
using std::endl;
using std::ofstream;
using std::vector;
using std::tuple;
using std::tie;
using std::make_tuple;
using std::shared_ptr;

#include "io.h"
#include "matrix.h"
#include "MyObject.h"

struct gearstat
{
    int xc;
    int yc;
    int min_x;
    int min_y;
    int max_x; 
    int max_y;
    double out_rad;
    double in_rad;
};


void bnr(const Image &im, Matrix<uint> &bin)
{
    int n_r = im.n_rows, n_c = im.n_cols;
    int r, g, b, h;
    h = 175;
    //int br;
    for(int i = 0; i < n_r; i++)
        for(int j = 0; j < n_c; j++)
        {
            // br = int(0.3 * r + 0.59 * g + 0.11 * b);
            tie(r, g, b) = im(i, j);
            if(r > h || g > h || b > h)
            	bin(i, j) = 1;
            else
                bin(i, j) = 0;
        }
}
void fill(const Matrix<uint> &bin, Matrix<uint> &lbl, int i, int j, uint counter)
{
    int n_r = bin.n_rows, n_c = bin.n_cols;

    if(i < 0 || i == n_r)
        return;
    if(j < 0 || j == n_c)
        return;
    if(lbl(i, j) || !bin(i, j))
        return;

    lbl(i, j) = counter;
    fill(bin, lbl, i + 1, j, counter);
    fill(bin, lbl, i, j + 1, counter);
    fill(bin, lbl, i - 1, j, counter);
    fill(bin, lbl, i, j - 1, counter);
}


int lb(const Matrix <uint> &bin, Matrix<uint> &lbl)
{
	int n_r = bin.n_rows, n_c = bin.n_cols;
	uint counter = 0;
	for(int i = 0; i < n_r; i++)
        for(int j = 0; j < n_c; j++)
            lbl(i, j) = 0;

    for(int j = 0; j < n_c; j++)
        for(int i = 0; i < n_r; i++)
            if(bin(i, j) && !lbl(i, j))
                fill(bin, lbl, i, j, ++counter);

    return counter;
}
tuple<int, vector<shared_ptr<IObject>>, Image>
repair_mechanism(const Image& im, const Image& im_g, Image& im_dst )
//repair_mechanism(const Image& im, const Image& im_g)
{
    auto object_array = vector<shared_ptr<IObject>>();
    int result_idx = 0;

	////////////////////////////BINARIZATION/////////////////////////////
	/////////////////////////////////////////////////////////////////////
	Matrix<uint> bin(im.n_rows, im.n_cols);
	Image binImg(im.n_rows, im.n_cols);

	bnr(im, bin);
	for (uint i = 0; i < im.n_rows; i++){
		for (uint j = 0; j < im.n_cols; j++){
			if (bin(i,j))
				binImg(i, j) = {170, 170, 170};
			else
				binImg(i, j) = {0, 0, 0};
		}
	}

	save_image(binImg, "bin.bmp");

	//------------------------------------------------------------------//	

	Matrix<uint> bin_g(im_g.n_rows, im_g.n_cols);
	Image binImg_g(im_g.n_rows, im_g.n_cols);

	bnr(im_g, bin_g);
	for (uint i = 0; i < im_g.n_rows; i++){
		for (uint j = 0; j < im_g.n_cols; j++){
			if (bin_g(i,j))
				binImg_g(i, j) = {170, 170, 170};
			else
				binImg_g(i, j) = {0, 0, 0};
		}
	}

	save_image(binImg_g, "bin_g.bmp");
	////////////////////////////LABELING/////////////////////////////////
	/////////////////////////////////////////////////////////////////////
	Matrix<uint> lbl(im.n_rows, im.n_cols);
	Image lblImg(im.n_rows, im.n_cols);
	
	uint n_obj = lb(bin, lbl);
	cout << "object num = " << n_obj << endl;

	for(uint k = 1; k <= n_obj; k++)
        for(uint i = 0; i < im.n_rows; i++)
            for(uint j = 0; j < im.n_cols; j++)
            	if(lbl(i, j) == k)
					switch ((k-1)%7){
						//RED
						case 0:
							lblImg(i, j) = {255, 0, 0};
							break;
						//ORANGE
						case 1:
							lblImg(i, j) = {255, 128, 0};
							break;
						//YELLOW
						case 2:
							lblImg(i, j) = {255, 255, 0};
							break;
						//GREEN
						case 3:
							lblImg(i, j) = {0, 255, 0};
							break;
						//LIGHT BLUE
						case 4:
							lblImg(i, j) = {0, 255, 255};
							break;
						//BLUE
						case 5:
							lblImg(i, j) = {0, 0, 255};
							break;
						//PURPLE
						case 6:
							lblImg(i, j) = {255, 0, 255};
							break;
						//WHITE
						default:
							lblImg(i, j) = {255, 255, 255};
							break;
					}

	//////////////////////////FIND CENTERS///////////////////////////////
	/////////////////////////////////////////////////////////////////////
	// repairing gear.xc = xI[0] / A[0];	
	vector <int> A(n_obj + 1), xI(n_obj + 1), yI(n_obj + 1);
	//	
	//Define integrals
	for(uint k = 1; k <= n_obj; k++)
    {
        A[k] = 0;
        xI[k] = 0;
        yI[k] = 0;
    }

    int n_r = lbl.n_rows, n_c = lbl.n_cols;
	//
	//Calculate integrals
    for(uint k = 1; k <= n_obj; k++)
    {
        for(int i = 0; i < n_r; i++)
            for(int j = 0; j < n_c; j++)
                if(lbl(i, j) == k)
                {
                    A[k]++;
                    xI[k] += i;
                    yI[k] += j;
                }
    }
	//
	//Find centers and surrender gear in square
	vector <gearstat> gear(n_obj + 1);
    for(uint k = 1; k <= n_obj; k++)
    {
        gear[k].max_x = gear[k].min_x = gear[k].xc = xI[k] / A[k];
        gear[k].max_y = gear[k].min_y = gear[k].yc = yI[k] / A[k];
        for(int i = 0; i < n_r; i++)
            for(int j = 0; j < n_c; j++)
                if(lbl(i, j) == k)
                {
                    if(i < gear[k].min_x)
                        gear[k].min_x = i;
                    if(i > gear[k].max_x)
                        gear[k].max_x = i;
                    if(j < gear[k].min_y)
                        gear[k].min_y = j;
                    if(j > gear[k].max_y)
                        gear[k].max_y = j;
                }
    }

	//------------------------------------------------------------------//	
	
    {
        A[0] = 0;
        xI[0] = 0;
        yI[0] = 0;
    }

    int ng_r = bin_g.n_rows, ng_c = bin_g.n_cols;
	//
	//Calculate integrals
    {
        for(int i = 0; i < ng_r; i++)
            for(int j = 0; j < ng_c; j++)
                if(bin_g(i, j))
                {
                    A[0]++;
                    xI[0] += i;
                    yI[0] += j;
                }
    }
	//
	//Find centers and surrender gear in square
    {
        gear[0].max_x = gear[0].min_x = gear[0].xc = xI[0] / A[0];
        gear[0].max_y = gear[0].min_y = gear[0].yc = yI[0] / A[0];
        for(int i = 0; i < ng_r; i++)
            for(int j = 0; j < ng_c; j++)
                if(bin_g(i, j))
                {
                    if(i < gear[0].min_x)
                        gear[0].min_x = i;
                    if(i > gear[0].max_x)
                        gear[0].max_x = i;
                    if(j < gear[0].min_y)
                        gear[0].min_y = j;
                    if(j > gear[0].max_y)
                        gear[0].max_y = j;
                }
    }
	
	//------------------------------------------------------------------//	
	
	//Draw Centers
    for(uint k = 1; k <= n_obj; k++)
        lblImg(gear[k].xc, gear[k].yc) = {0, 0, 0};
	//Draw limits
    for(uint k = 1; k <= n_obj; k++)
    {
        for(int i = gear[k].min_x, j = gear[k].min_y; i <= gear[k].max_x; i++)
            lblImg(i, j) = {255, 0, 170};
        for(int i = gear[k].min_x, j = gear[k].min_y; j <= gear[k].max_y; j++)
            lblImg(i, j) = {255, 0, 170};
        for(int i = gear[k].min_x, j = gear[k].max_y; i <= gear[k].max_x; i++)
            lblImg(i, j) = {255, 0, 170};
        for(int i = gear[k].max_x, j = gear[k].min_y; j <= gear[k].max_y; j++)
            lblImg(i, j) = {255, 0, 170};
    }
	//
	save_image(lblImg, "labeling.bmp");	
	//////////////////////////////RADIUS/////////////////////////////////
	/////////////////////////////////////////////////////////////////////
	double rad;
    for(uint k = 1; k <= n_obj; k++)
    {
        gear[k].out_rad = 0;
        gear[k].in_rad = sqrt(pow(gear[k].max_x - gear[k].min_x, 2) + pow(gear[k].max_y - gear[k].min_y, 2));
        for(int i = gear[k].min_x; i <= gear[k].max_x; i++)
        {
            for(int j = gear[k].min_y; j <= gear[k].max_y; j++)
            {
                rad = sqrt(pow(i - gear[k].xc, 2) + pow(j - gear[k].yc, 2));
                if(lbl(i, j) == k && rad > gear[k].out_rad)
                {
                    gear[k].out_rad = floor(rad);
                }

                if(lbl(i, j) == 0 && rad < gear[k].in_rad)
                {
                    gear[k].in_rad = floor(rad);
                }
            }
        }
    }

	//------------------------------------------------------------------//	

	{
        gear[0].out_rad = 0;
        gear[0].in_rad = sqrt(pow(gear[0].max_x - gear[0].min_x, 2) + pow(gear[0].max_y - gear[0].min_y, 2));
        for(int i = gear[0].min_x; i <= gear[0].max_x; i++)
        {
            for(int j = gear[0].min_y; j <= gear[0].max_y; j++)
            {
                rad = sqrt(pow(i - gear[0].xc, 2) + pow(j - gear[0].yc, 2));
                if(bin_g(i, j) == 1 && rad > gear[0].out_rad)
                {
                    gear[0].out_rad = floor(rad);
                }

                if(bin_g(i, j) == 0 && rad < gear[0].in_rad)
                {
                    gear[0].in_rad = floor(rad);
                }
            }
        }
    }
    uint target = 0;
    for(uint k = 1; k <= n_obj; k++)
    {
    	if(static_cast<int>(gear[k].out_rad) == static_cast<int>(gear[k].in_rad))
    	{
    		target = k;
    		break;
    	}	
    }
    
	
    int diff1 = abs(gear[target].xc - gear[0].xc);
    int diff2 = abs(gear[target].yc - gear[0].xc);
    uint r, g, b;
    for(int i = gear[0].min_x; i <= gear[0].max_x; i++)
    {
        for(int j = gear[0].min_y; j <= gear[0].max_y; j++)
        {
        	tie(r, g, b) = im_g(i, j);
        	if(bin_g(i, j) == 1)
        	{
        		if(!(lbl(i + diff1, j + diff2) == 0 || lbl(i + diff1, j + diff2) == target))
        			result_idx = 1;
        		im_dst(i+diff1, j+diff2) = {r, g, b};
        	}
        }
    }
	
	ofstream fout("info.txt");
	fout << "Gears count: " << result_idx << endl;
		for(uint k = 0; k <= n_obj; k++){
			fout << "Gear #" << k << endl;
			fout << "xc = " << gear[k].xc << endl;		
			fout << "yc = " << gear[k].yc << endl;
			fout << "R = "<< gear[k].out_rad << endl;
			fout << "r = " << gear[k].in_rad << endl;
			fout << endl << endl << endl;
		}
	
		
    return make_tuple(result_idx, object_array, im.deep_copy());
}

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        cout << "Usage: " << endl << argv[0]
             << " <in_image.bmp> <out_image.bmp> <gear.bmp> <out_result.txt>" << endl;
        return 0;
    }

    try 
	{
        Image src_image = load_image(argv[1]);///ORIGINAL IMAGE
		Image gear_image = load_image(argv[3]);///GEAR IMAGE
        ofstream fout(argv[4]);///OUTPUT TEXT

        vector<shared_ptr<IObject>> object_array;
        Image dst_image;
        Image im_compl = src_image;
        
        int result_idx;
        tie(result_idx, object_array, dst_image) = repair_mechanism(src_image, gear_image, im_compl);
        //tie(result_idx, object_array, dst_image) = repair_mechanism(src_image, gear_image);
        //save_image(dst_image, argv[2]);////OUTPUT IMAGE
        save_image(im_compl, argv[2]);////OUTPUT IMAGE
		
			
        //fout << object_array.size() << endl;
        for (const auto &obj : object_array)
            obj->Write(fout);

    } catch (const string &s) {
        cerr << "Error: " << s << endl;
        return 1;
    }
}
/*


tuple<int, vector<shared_ptr<IObject>>, Image>
repair_mechanism(const Image& im)
{
    // Base: return array of found objects and index of the correct gear
    // Bonus: return additional parameters of gears


	/// bin array with zeros and ones: ones for gears and zeros for background
	Matrix<uint> bin(im.n_rows, im.n_cols),
	///lbl array with numbers and zeros like binarisation to make a count of gears
	lbl(im.n_rows, im.n_cols);

    Image binImg(im.n_rows, im.n_cols),/// Image binarisation
	lblImg(im.n_rows, im.n_cols);/// Image labeling
-
    auto object_array = vector<shared_ptr<IObject>>();
	uint n_obj = 0; n_obj++;
    int result_idx = 0;

	bnr(im, bin);
	
	
	for (int x = 0; x < im.n_rows; x++){
		for (int y = 0; y < im_n_cols; y++)
			cout << bin(x,y);
		cout << endl;
	}
	

    return make_tuple(result_idx, object_array, im.deep_copy());
}



*/
