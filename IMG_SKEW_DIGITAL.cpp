#include <scamp7.hpp>
#include <math.h>

#include "IMG_ROTATION_DIGITAL.hpp"

using namespace SCAMP7_PE;

namespace IMGTF
{
	namespace SKEW
	{
		namespace DIGITAL
		{
			void SKEWX(dreg_t reg,int skew_mag, bool skew_anti_clockwise,double offset)
			{
				if(skew_mag == 0)
				{
					return;
				}

				const int scalar_hack = 1000;
				int step_size = (int)(scalar_hack*127.0/skew_mag);

				if(!dreg_eql(reg,S6))
				{
					scamp7_dynamic_kernel_begin();
						MOV(S6,reg);
					scamp7_dynamic_kernel_end();
				}

				scamp7_kernel_begin()
					CLR(RS,RN);
					SET(RF);
					MOV(RM,S6);
				scamp7_kernel_end();

				if(!skew_anti_clockwise)
				{
					for(int n = offset*step_size  ; n < 127*scalar_hack ; n+= step_size)
					{
						int in = n/scalar_hack;

						scamp7_load_rect(RW,0,0,in,255);
						scamp7_load_rect(RE,255-in,0,255,255);
						scamp7_kernel_begin()
							DNEWS0(S6,RM);
							OR(RF,RW,RE);
							MOV(RM,S6);
						scamp7_kernel_end();
					}
				}
				else
				{
					for(int n = offset*step_size  ; n < 127*scalar_hack ; n+= step_size)
					{
						int in = n/scalar_hack;

						scamp7_load_rect(RE,0,0,in,255);
						scamp7_load_rect(RW,255-in,0,255,255);
						scamp7_kernel_begin();
							DNEWS0(S6,RM);
							OR(RF,RW,RE);
							MOV(RM,S6);
						scamp7_kernel_end();
					}
				}

				scamp7_kernel_begin()
					MOV(S6,RM);
				scamp7_kernel_end();

				if(!dreg_eql(reg,S6))
				{
					scamp7_dynamic_kernel_begin();
						MOV(reg,S6);
					scamp7_dynamic_kernel_end();
				}
			}

			void SKEWX_TAN_RAD(dreg_t reg,double tan_of_angle,double offset)
			{
				double skew_mag = 127.0*tan_of_angle;
				SKEWX( reg,(int)fabs(skew_mag),skew_mag > 0 ? true : false, offset);
			}

			void SKEWX_RAD(dreg_t reg,double angle,double offset)
			{
				double skew_mag = 127.0*tan_approx3(angle);
				SKEWX(reg,(int)fabs(skew_mag),skew_mag > 0 ? true : false, offset);
			}

			void SKEWX_DEG(dreg_t reg,double angle,double offset)
			{
				double angle_in_radians = M_PI*angle/180.0;
				SKEWX_RAD(reg,angle_in_radians, offset);
			}

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			void SKEWY(dreg_t reg,int skew_mag, bool skew_anti_clockwise,double offset)
			{
				if(skew_mag == 0)
				{
					return;
				}

				const int scalar_hack = 1000;
				int step_size = (int)(scalar_hack*127.0/skew_mag);

				if(!dreg_eql(reg,S6))
				{
					scamp7_dynamic_kernel_begin();
						MOV(S6,reg);
					scamp7_dynamic_kernel_end();
				}

				scamp7_kernel_begin()
					CLR(RW,RE);
					SET(RF);
					MOV(RM,S6);
				scamp7_kernel_end();

				if(!skew_anti_clockwise)
				{
					for(int n = offset*step_size  ; n < 127*scalar_hack ; n+= step_size)
					{
						int in = n/scalar_hack;

						scamp7_load_rect(RN,0,0,255,in);
						scamp7_load_rect(RS,0,255-in,255,255);
						scamp7_kernel_begin()
							DNEWS0(S6,RM);
							OR(RF,RS,RN);
							MOV(RM,S6);
						scamp7_kernel_end();
					}
				}
				else
				{
					for(int n = offset*step_size  ; n < 127*scalar_hack ; n+= step_size)
					{
						int in = n/scalar_hack;

						scamp7_load_rect(RS,0,0,255,in);
						scamp7_load_rect(RN,0,255-in,255,255);
						scamp7_kernel_begin()
							DNEWS0(S6,RM);
							OR(RF,RS,RN);
							MOV(RM,S6);
						scamp7_kernel_end();
					}
				}

				scamp7_kernel_begin()
					MOV(S6,RM);
				scamp7_kernel_end();

				if(!dreg_eql(reg,S6))
				{
					scamp7_dynamic_kernel_begin();
						MOV(reg,S6);
					scamp7_dynamic_kernel_end();
				}
			}

			void SKEWY_TAN_RAD(dreg_t reg,double tan_of_angle,double offset)
			{
				double skew_mag = 127.0*tan_of_angle;
				SKEWY( reg,(int)fabs(skew_mag),skew_mag > 0 ? true : false, offset);
			}

			void SKEWY_RAD(dreg_t reg,double angle,double offset)
			{
				double skew_mag = 127.0*tan_approx3(angle);
				SKEWY(reg,(int)fabs(skew_mag),skew_mag > 0 ? true : false, offset);
			}

			void SKEWY_DEG(dreg_t reg,double angle,double offset)
			{
				double angle_in_radians = M_PI*angle/180.0;
				SKEWY_RAD(reg,angle_in_radians,offset);
			}

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			void STEP_SKEWX_CW_S6(int step_number)
			{
				unsigned char i = reverse_byte(step_number)/2;

				scamp7_load_rect(RW,0,0,i,255);
				scamp7_load_rect(RE,255-i,0,255,255);

				scamp7_kernel_begin()
					CLR(RS,RN);
					SET(RF);
					MOV(RM,S6);
					DNEWS0(S6,RM);
					OR(RF,RW,RE);
					MOV(RM,S6);
					MOV(S6,RM);
				scamp7_kernel_end();
				return;
			}

			void STEP_SKEWX_ACW_S6(int step_number)
			{
				unsigned char i = reverse_byte(step_number)/2;

				scamp7_load_rect(RE,0,0,i,255);
				scamp7_load_rect(RW,255-i,0,255,255);

				scamp7_kernel_begin()
					CLR(RS,RN);
					SET(RF);
					MOV(RM,S6);
					DNEWS0(S6,RM);
					OR(RF,RW,RE);
					MOV(RM,S6);
					MOV(S6,RM);
				scamp7_kernel_end();
				return;
			}

			void STEP_SKEWY_CW_S6(int step_number)
			{
				unsigned char i = reverse_byte(step_number)/2;

				scamp7_load_rect(RN,0,0,255,i);
				scamp7_load_rect(RS,0,255-i,255,255);

				scamp7_kernel_begin()
					CLR(RW,RE);
					SET(RF);
					MOV(RM,S6);
					DNEWS0(S6,RM);
					OR(RF,RS,RN);
					MOV(RM,S6);
					MOV(S6,RM);
				scamp7_kernel_end();
				return;
			}

			void STEP_SKEWY_ACW_S6(int step_number)
			{
				unsigned char i = reverse_byte(step_number)/2;

				scamp7_load_rect(RS,0,0,255,i);
				scamp7_load_rect(RN,0,255-i,255,255);

				scamp7_kernel_begin()
					CLR(RW,RE);
					SET(RF);
					MOV(RM,S6);
					DNEWS0(S6,RM);
					OR(RF,RS,RN);
					MOV(RM,S6);
					MOV(S6,RM);
				scamp7_kernel_end();
				return;
			}
		}
	}
}




