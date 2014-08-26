#include "registration.h"



KinectRegistration::KinectRegistration(bool mirror){

	if (mirror)
		DEPTH_MIRROR_X = !DEPTH_MIRROR_X;
	
	freenect_video_format requested_format = FREENECT_VIDEO_RGB;
    device = &myfreenect.createDevice<Freenect::FreenectDevice>(0);
    //device->setDepthFormat(FREENECT_DEPTH_REGISTERED);
    device->setVideoFormat(requested_format);
    dev = const_cast<freenect_device*>(device->getDevice());
    reg = freenect_copy_registration(dev);
    zpi = &(reg.zero_plane_info);
    regdata = &(reg.reg_info);
}

void KinectRegistration::SetMirror(){
		DEPTH_MIRROR_X = !DEPTH_MIRROR_X;
	}

void KinectRegistration::freenect_create_dxdy_tables(double* reg_x_table, double* reg_y_table, int32_t resolution_x, int32_t resolution_y )
{

	int64_t AX6 = regdata->ax;
	int64_t BX6 = regdata->bx;
	int64_t CX2 = regdata->cx;
	int64_t DX2 = regdata->dx;

	int64_t AY6 = regdata->ay;
	int64_t BY6 = regdata->by;
	int64_t CY2 = regdata->cy;
	int64_t DY2 = regdata->dy;

	// don't merge the shift operations - necessary for proper 32-bit clamping of extracted values
	int64_t dX0 = (regdata->dx_start << 13) >> 4;
	int64_t dY0 = (regdata->dy_start << 13) >> 4;

	int64_t dXdX0 = (regdata->dxdx_start << 11) >> 3;
	int64_t dXdY0 = (regdata->dxdy_start << 11) >> 3;
	int64_t dYdX0 = (regdata->dydx_start << 11) >> 3;
	int64_t dYdY0 = (regdata->dydy_start << 11) >> 3;

	int64_t dXdXdX0 = (regdata->dxdxdx_start << 5) << 3;
	int64_t dYdXdX0 = (regdata->dydxdx_start << 5) << 3;
	int64_t dYdXdY0 = (regdata->dydxdy_start << 5) << 3;
	int64_t dXdXdY0 = (regdata->dxdxdy_start << 5) << 3;
	int64_t dYdYdX0 = (regdata->dydydx_start << 5) << 3;
	int64_t dYdYdY0 = (regdata->dydydy_start << 5) << 3;

	int32_t row,col,tOffs = 0;

	for (row = 0 ; row < resolution_y ; row++) {

		dXdXdX0 += CX2;

		dXdX0   += dYdXdX0 >> 8;
		dYdXdX0 += DX2;

		dX0     += dYdX0 >> 6;
		dYdX0   += dYdYdX0 >> 8;
		dYdYdX0 += BX6;

		dXdXdY0 += CY2;

		dXdY0   += dYdXdY0 >> 8;
		dYdXdY0 += DY2;

		dY0     += dYdY0 >> 6;
		dYdY0   += dYdYdY0 >> 8;
		dYdYdY0 += BY6;

		int64_t coldXdXdY0 = dXdXdY0, coldXdY0 = dXdY0, coldY0 = dY0;

		int64_t coldXdXdX0 = dXdXdX0, coldXdX0 = dXdX0, coldX0 = dX0;

		for (col = 0 ; col < resolution_x ; col++, tOffs++) {
			reg_x_table[tOffs] = coldX0 * (1.0/(1<<17));
			reg_y_table[tOffs] = coldY0 * (1.0/(1<<17));

			coldX0     += coldXdX0 >> 6;
			coldXdX0   += coldXdXdX0 >> 8;
			coldXdXdX0 += AX6;

			coldY0     += coldXdY0 >> 6;
			coldXdY0   += coldXdXdY0 >> 8;
			coldXdXdY0 += AY6;
		}
	}
}

void KinectRegistration::init(int depthx,int depthy, int depthxoff, int depthyoff)
{
	DEPTH_X_RES = depthx;
	DEPTH_Y_RES = depthy;


	std::vector<double> dx(depthx*depthy);
	std::vector<double> dy(depthx*depthy);

	freenect_create_dxdy_tables(&dx[0],&dy[0],depthx,depthy);

	registration_table.resize(depthx*depthy);

	int index = 0;
	for (int y = 0; y < depthy; y++) {
		for (int x = 0; x < depthx; x++, index++) {
			double new_x = x + dx[index] + depthxoff;
			double new_y = y + dy[index] + depthyoff;

			if ((new_x < 0) || (new_y < 0) || (new_x >= depthx) || (new_y >= depthy))
				new_x = 2 * depthx; // intentionally set value outside image bounds

			registration_table[index].x = new_x * REG_X_VAL_SCALE;
			registration_table[index].y = new_y;
		}
	}

	uint32_t i,x_scale = DEPTH_SENSOR_X_RES / depthx;

	double pixel_size = 1.0 / (zpi->reference_pixel_size * x_scale * S2D_PIXEL_CONST);
	double pixels_between_rgb_and_ir_cmos = zpi->dcmos_rcmos_dist * pixel_size * S2D_PIXEL_CONST;
	double reference_distance_in_pixels = zpi->reference_distance * pixel_size * S2D_PIXEL_CONST;

	depth_to_rgb_shift.resize(DEPTH_MAX_METRIC_VALUE);
	for (i = 0; i < DEPTH_MAX_METRIC_VALUE; i++) {
		double current_depth_in_pixels = i * pixel_size;
		depth_to_rgb_shift[i] = (( pixels_between_rgb_and_ir_cmos * (current_depth_in_pixels - reference_distance_in_pixels) / current_depth_in_pixels) + S2D_CONST_OFFSET) * REG_X_VAL_SCALE;
	}
}

void KinectRegistration::apply_registration( uint16_t * input_raw_mm, uint16_t* output_mm)
{
	const int DEPTH_X_RES = this->DEPTH_X_RES;
	const int DEPTH_Y_RES = this->DEPTH_Y_RES;

	int n = DEPTH_X_RES*DEPTH_Y_RES;
	uint32_t target_offset = DEPTH_Y_RES * reg.reg_pad_info.start_lines;

	std::fill(output_mm,output_mm + n, DEPTH_NO_MM_VALUE);

	for (int y = 0; y < DEPTH_Y_RES; y++) {
		for (int x = 0; x < DEPTH_X_RES; x++) {


			uint16_t metric_depth  = input_raw_mm[x+y*DEPTH_X_RES];
			// so long as the current pixel has a depth value
			if (metric_depth == DEPTH_NO_MM_VALUE) continue;
			if (metric_depth >= DEPTH_MAX_METRIC_VALUE) continue;

			// calculate the new x and y location for that pixel
			// using registration_table for the basic rectification
			// and depth_to_rgb_shift for determining the x shift
			uint32_t reg_index = DEPTH_MIRROR_X ? ((y + 1) * DEPTH_X_RES - x - 1) : (y * DEPTH_X_RES + x);
			uint32_t nx = (registration_table[reg_index].x + depth_to_rgb_shift[metric_depth]) / REG_X_VAL_SCALE;
			uint32_t ny =  registration_table[reg_index].y;
			// ignore anything outside the image bounds
			if (nx >= DEPTH_X_RES) continue;

			// convert nx, ny to an index in the depth image array
			uint32_t target_index = (DEPTH_MIRROR_X ? ((ny + 1) * DEPTH_X_RES - nx - 1) : (ny * DEPTH_X_RES + nx)) - target_offset;
			// get the current value at the new location
			uint16_t current_depth = output_mm[target_index];

			// make sure the new location is empty, or the new value is closer
			if ((current_depth == DEPTH_NO_MM_VALUE) || (current_depth > metric_depth)) {
				output_mm[target_index] = metric_depth; // always save depth at current location

				if(DENSE_REGISTRATION)
				{
					// if we're not on the first row, or the first column
					if ((nx > 0) && (ny > 0)) {
						output_mm[target_index - DEPTH_X_RES    ] = metric_depth; // save depth at (x,y-1)
						output_mm[target_index - DEPTH_X_RES - 1] = metric_depth; // save depth at (x-1,y-1)
						output_mm[target_index               - 1] = metric_depth; // save depth at (x-1,y)
					} else if (ny > 0) {
						output_mm[target_index - DEPTH_X_RES] = metric_depth; // save depth at (x,y-1)
					} else if (nx > 0) {
						output_mm[target_index - 1] = metric_depth; // save depth at (x-1,y)
					}
				}
			}
		}
	}
}
