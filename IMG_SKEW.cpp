#include "IMG_SKEW.hpp"
using namespace SCAMP7_PE;

namespace IMGTF
{
	namespace SKEW
	{
		namespace ANALOG
		{
			void SKEWX(areg_t reg,int skew_mag, bool skew_anti_clockwise,double offset)
			{
				if(skew_mag == 0)
				{
					return;
				}

				scamp7_dynamic_kernel_begin();
					mov(F,reg);
				scamp7_dynamic_kernel_end();

				const int scalar_hack = 1000;
				int step_size = (int)(scalar_hack*127.0/skew_mag);
				if(!skew_anti_clockwise)
				{
					for(int n = offset*step_size  ; n < 127*scalar_hack ; n+= step_size)
					{
						int in = n/scalar_hack;
						scamp7_load_rect(RS,0,0,in,255);
						scamp7_load_rect(RW,255-in,0,255,255);

						scamp7_kernel_begin();
//							mov(E,F);
//
//							WHERE(RS);
//								movx(F,E,west);
//							WHERE(RW);
//								movx(F,E,east);
//							all();

							bus(NEWS,F);
							WHERE(RS);
								bus(F,XW);
							WHERE(RW);
								bus(F,XE);
							all();
						 scamp7_kernel_end();
					}
				}
				else
				{
					for(int n = offset*step_size  ; n < 127*scalar_hack ; n+= step_size)
					{
						int in = n/scalar_hack;
						scamp7_load_rect(RS,0,0,in,255);
						scamp7_load_rect(RW,255-in,0,255,255);

						scamp7_kernel_begin();
//							mov(E,F);
//
//							WHERE(RS);
//								movx(F,E,east);
//							WHERE(RW);
//								movx(F,E,west);
//							all();

							bus(NEWS,F);
							WHERE(RS);
								bus(F,XE);
							WHERE(RW);
								bus(F,XW);
							all();
						 scamp7_kernel_end();
					}
				}

				scamp7_dynamic_kernel_begin();
					mov(reg,F);
				scamp7_dynamic_kernel_end();
			}

			void SKEWX_TAN_RAD(areg_t reg,double tan_of_angle,double offset)
			{
				double skew_mag = 127.0*tan_of_angle;
				SKEWX(reg,(int)fabs(skew_mag),skew_mag > 0 ? true : false, offset);
			}

			void SKEWX_RAD(areg_t reg,double angle,double offset)
			{
				double skew_mag = 127*tan_approx3(angle);
				SKEWX(reg,(int)fabs(skew_mag),skew_mag > 0 ? true : false, offset);
			}

			void SKEWX_DEG(areg_t reg,double angle,double offset)
			{
				double angle_in_radians = M_PI*angle/180.0;
				SKEWX_RAD(reg,angle_in_radians, offset);
			}

			/////////////////////////////////////////////////////////////////////////

			void SKEWY(areg_t reg,int skew_mag, bool skew_anti_clockwise,double offset)
			{
				if(skew_mag == 0)
				{
					return;
				}

				scamp7_dynamic_kernel_begin();
					mov(F,reg);
				scamp7_dynamic_kernel_end();

				const int scalar_hack = 1000;
				int step_size = (int)(scalar_hack*127.0/skew_mag);

				if(!skew_anti_clockwise)
				{
					for(int n = offset*step_size  ; n < 127*scalar_hack ; n+= step_size)
					{
						int in = n/scalar_hack;
						scamp7_load_rect(RS,0,0,255,in);
						scamp7_load_rect(RW,0,255-in,255,255);

						scamp7_kernel_begin();
//							mov(E,F);
//
//							WHERE(RS);
//								movx(F,E,north);
//							WHERE(RW);
//								movx(F,E,south);
//							all();

							bus(NEWS,F);
							WHERE(RS);
								bus(F,XN);
							WHERE(RW);
								bus(F,XS);
							all();
						scamp7_kernel_end();
					}
				}
				else
				{
					for(int n = offset*step_size  ; n < 127*scalar_hack ; n+= step_size)
					{
						int in = n/scalar_hack;
						scamp7_load_rect(RS,0,0,255,in);
						scamp7_load_rect(RW,0,255-in,255,255);

						scamp7_kernel_begin();
//							mov(E,F);
//
//							WHERE(RS);
//								movx(F,E,south);
//							WHERE(RW);
//								movx(F,E,north);
//							all();

							bus(NEWS,F);
							WHERE(RS);
								bus(F,XS);
							WHERE(RW);
								bus(F,XN);
							all();
						scamp7_kernel_end();
					}
				}

				scamp7_dynamic_kernel_begin();
					mov(reg,F);
				scamp7_dynamic_kernel_end();
			}

			void SKEWY_TAN_RAD(areg_t reg,double tan_of_angle,double offset)
			{
				double skew_mag = 127.0*tan_of_angle;
				SKEWY(reg,(int)fabs(skew_mag),skew_mag > 0 ? true : false, offset);
			}

			void SKEWY_RAD(areg_t reg,double angle,double offset)
			{
				double skew_mag = 127.0*tan_approx3(angle);
				SKEWY(reg,(int)fabs(skew_mag),skew_mag > 0 ? true : false, offset);
			}

			void SKEWY_DEG(areg_t reg,double angle,double offset)
			{
				double angle_in_radians = M_PI*angle/180.0;
				SKEWY_RAD(reg,angle_in_radians,offset);
			}

			/////////////////////////////////////////////////////////////////////////

			void STEP_SKEWX_CW_F(int step_number)
			{
				unsigned char i = reverse_byte(step_number)/2;

				scamp7_load_rect(RS,0,0,i,255);
				scamp7_load_rect(RW,255-i,0,255,255);

				scamp7_kernel_begin();
					bus(NEWS,F);

					WHERE(RS);
						bus(F,XW);
					all();

					WHERE(RW);
						bus(F,XE);
					all();
				scamp7_kernel_end();
				return;
			}

			void STEP_SKEWX_ACW_F(int step_number)
			{
				unsigned char i = reverse_byte(step_number)/2;

				scamp7_load_rect(RS,0,0,i,255);
				scamp7_load_rect(RW,255-i,0,255,255);

				scamp7_kernel_begin();
					bus(NEWS,F);

					WHERE(RS);
						bus(F,XE);
					all();

					WHERE(RW);
						bus(F,XW);
					all();
				scamp7_kernel_end();
				return;
			}

			void STEP_SKEWY_CW_F(int step_number)
			{
				unsigned char i = reverse_byte(step_number)/2;

				scamp7_load_rect(RS,0,0,255,i);
				scamp7_load_rect(RW,0,255-i,255,255);

				scamp7_kernel_begin();
					bus(NEWS,F);

					WHERE(RS);
						bus(F,XN);
					all();

					WHERE(RW);
						bus(F,XS);
					all();
				scamp7_kernel_end();
				return;
			}

			void STEP_SKEWY_ACW_F(int step_number)
			{
				unsigned char i = reverse_byte(step_number)/2;

				scamp7_load_rect(RS,0,0,255,i);
				scamp7_load_rect(RW,0,255-i,255,255);

				scamp7_kernel_begin();
					bus(NEWS,F);

					WHERE(RS);
						bus(F,XS);
					all();

					WHERE(RW);
						bus(F,XN);
					all();
				scamp7_kernel_end();
				return;
			}

		}
	}
}

