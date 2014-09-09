#include <libfreenect/libfreenect.hpp>
#include <libfreenect/libfreenect_registration.h>
#include <vector>

class KinectRegistration
{
public:

	struct xypair {
		int32_t x,y;
	};

	std::vector<xypair> registration_table;
	std::vector<int32_t> depth_to_rgb_shift;
	int32_t DEPTH_X_RES,DEPTH_Y_RES; 

	int DEPTH_MAX_METRIC_VALUE = 10000;
	int DEPTH_NO_MM_VALUE = 10000;

	int REG_X_VAL_SCALE = 256; 
	int S2D_PIXEL_CONST = 10;
	double S2D_CONST_OFFSET = 0.375;
	int DEPTH_SENSOR_X_RES = 1280; 
	bool DEPTH_MIRROR_X = false;
	bool DENSE_REGISTRATION = false;

	freenect_reg_info * regdata = 0;
	freenect_zero_plane_info * zpi = 0;
	Freenect::Freenect myfreenect;
    Freenect::FreenectDevice * device = 0;
    freenect_registration reg;
    freenect_device * dev = 0;

	KinectRegistration(bool mirror);

	void init(int depthx ,int depthy , int depthxoff, int depthyoff);

	void apply_registration( uint16_t * input_raw_mm, uint16_t* output_mm);

	void freenect_create_dxdy_tables(double* reg_x_table, double* reg_y_table, int32_t resolution_x, int32_t resolution_y );

	void SetMirror();
	
};

