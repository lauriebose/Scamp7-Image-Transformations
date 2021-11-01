#include "IMG_SCALING_DIGITAL.hpp"

static const int scaling_rowcol_order[] = {
		64, 32, 98, 16, 83, 50, 118, 8, 77, 43, 113, 26, 97, 62, 4, 77, 41, 114, 23, 97, 61, 14, 90, 53, 34, 113, 74, 2,
		83, 43, 125, 23, 105, 64, 13, 97, 55, 35, 121, 79, 8, 97, 52, 31, 121, 76, 20, 113, 67, 44, 92, 2, 98, 50, 26, 125,
		76, 14, 115, 65, 40, 92, 8, 113, 61, 35, 89, 22, 77, 50, 106, 5, 118, 62, 34, 91, 20, 78, 50, 110, 13, 74, 44, 106,
		29, 92, 61, 125,
};


namespace IMGTF
{
	namespace SCALING
	{
		namespace DIGITAL
		{

			void STEP_SCALE_UP_S6(int step_number)
			{
				unsigned char i = scaling_rowcol_order[step_number];

				scamp7_kernel_begin();
					SET(RF);
					MOV(RM,S6);
				scamp7_kernel_end();


				scamp7_kernel_begin()
							CLR(RS,RN);
				scamp7_kernel_end();
				scamp7_load_rect(RW,0,0,255,128-i);
				scamp7_load_rect(RE,0,i+128,255,255);

				scamp7_kernel_begin();
					DNEWS0(S6,RM);
					OR(RF,RW,RE);
					MOV(RM,S6);
				scamp7_kernel_end();

				scamp7_kernel_begin()
							CLR(RW,RE);
				scamp7_kernel_end();
				scamp7_load_rect(RS,0,0,128-i,255);
				scamp7_load_rect(RN,i+128,0,255,255);

				scamp7_kernel_begin();
					DNEWS0(S6,RM);
					OR(RF,RS,RN);
					MOV(RM,S6);
				scamp7_kernel_end();


				scamp7_kernel_begin();
					MOV(S6,RM);
				scamp7_kernel_end();
				return;
			}

			void STEP_SCALE_DOWN_S6(int step_number)
			{
				unsigned char i = scaling_rowcol_order[step_number];

				scamp7_kernel_begin();
					SET(RF);
					MOV(RM,S6);
				scamp7_kernel_end();

				scamp7_kernel_begin()
							CLR(RS,RN);
				scamp7_kernel_end();
				scamp7_load_rect(RE,0,0,255,128-i);
				scamp7_load_rect(RW,0,i+128,255,255);

				scamp7_kernel_begin();
					DNEWS0(S6,RM);
					OR(RF,RW,RE);
					MOV(RM,S6);
				scamp7_kernel_end();

				scamp7_kernel_begin()
							CLR(RW,RE);
				scamp7_kernel_end();
				scamp7_load_rect(RN,0,0,128-i,255);
				scamp7_load_rect(RS,i+128,0,255,255);

				scamp7_kernel_begin();
					DNEWS0(S6,RM);
					OR(RF,RS,RN);
					MOV(RM,S6);
				scamp7_kernel_end();

				scamp7_kernel_begin();
					MOV(S6,RM);
				scamp7_kernel_end();
				return;
			}

			void STEP_SCALE_UPY_S6(int step_number)
			{
				unsigned char i = scaling_rowcol_order[step_number];

				scamp7_kernel_begin();
					SET(RF);
					MOV(RM,S6);
				scamp7_kernel_end();

				scamp7_kernel_begin()
							CLR(RW,RE);
				scamp7_kernel_end();
				scamp7_load_rect(RS,0,0,128-i,255);
				scamp7_load_rect(RN,i+128,0,255,255);

				scamp7_kernel_begin();
					DNEWS0(S6,RM);
					OR(RF,RS,RN);
					MOV(RM,S6);
				scamp7_kernel_end();

				scamp7_kernel_begin();
					MOV(S6,RM);
				scamp7_kernel_end();
				return;
			}

			void STEP_SCALE_DOWNY_S6(int step_number)
			{
				unsigned char i = scaling_rowcol_order[step_number];

				scamp7_kernel_begin();
					SET(RF);
					MOV(RM,S6);
				scamp7_kernel_end();

				scamp7_kernel_begin()
							CLR(RW,RE);
				scamp7_kernel_end();
				scamp7_load_rect(RN,0,0,128-i,255);
				scamp7_load_rect(RS,i+128,0,255,255);

				scamp7_kernel_begin();
					DNEWS0(S6,RM);
					OR(RF,RS,RN);
					MOV(RM,S6);
				scamp7_kernel_end();

				scamp7_kernel_begin();
					MOV(S6,RM);
				scamp7_kernel_end();
				return;
			}

			void STEP_SCALE_UPX_S6(int step_number)
			{
				unsigned char i = scaling_rowcol_order[step_number];

				scamp7_kernel_begin();
					SET(RF);
					MOV(RM,S6);
				scamp7_kernel_end();

				scamp7_kernel_begin()
					CLR(RS,RN);
				scamp7_kernel_end();
				scamp7_load_rect(RW,0,0,255,128-i);
				scamp7_load_rect(RE,0,i+128,255,255);

				scamp7_kernel_begin();
					DNEWS0(S6,RM);
					OR(RF,RW,RE);
					MOV(RM,S6);
				scamp7_kernel_end();

				scamp7_kernel_begin();
					MOV(S6,RM);
				scamp7_kernel_end();
				return;
			}

			void STEP_SCALE_DOWNX_S6(int step_number)
			{
				unsigned char i = scaling_rowcol_order[step_number];

				scamp7_kernel_begin();
					SET(RF);
					MOV(RM,S6);
				scamp7_kernel_end();

				scamp7_kernel_begin()
					CLR(RS,RN);
				scamp7_kernel_end();
				scamp7_load_rect(RE,0,0,255,128-i);
				scamp7_load_rect(RW,0,i+128,255,255);

				scamp7_kernel_begin();
					DNEWS0(S6,RM);
					OR(RF,RW,RE);
					MOV(RM,S6);
				scamp7_kernel_end();

				scamp7_kernel_begin();
					MOV(S6,RM);
				scamp7_kernel_end();
				return;
			}

			void SCALE_Y(dreg_t reg,int scaling_mag,bool scale_down)
			{
				if(!dreg_eql(reg,S6))
				{
					scamp7_dynamic_kernel_begin();
						MOV(S6,reg);
					scamp7_dynamic_kernel_end();
				}
				if(!scale_down)
				{
					for(unsigned char n = 0 ; n < scaling_mag ; n++)
					{
						STEP_SCALE_UPY_S6(n);
					}
				}
				else
				{
					for(unsigned char n = 0 ; n < scaling_mag ; n++)
					{
						STEP_SCALE_DOWNY_S6(n);
					}
				}
				if(!dreg_eql(reg,S6))
				{
					scamp7_dynamic_kernel_begin();
						MOV(reg,S6);
					scamp7_dynamic_kernel_end();
				}
			}


			void SCALE_X(dreg_t reg,int scaling_mag,bool scale_down)
			{
				if(!dreg_eql(reg,S6))
				{
					scamp7_dynamic_kernel_begin();
						MOV(S6,reg);
					scamp7_dynamic_kernel_end();
				}
				if(!scale_down)
				{
					for(unsigned char n = 0 ; n < scaling_mag ; n++)
					{
						STEP_SCALE_UPX_S6(n);
					}
				}
				else
				{
					for(unsigned char n = 0 ; n < scaling_mag ; n++)
					{
						STEP_SCALE_DOWNX_S6(n);
					}
				}
				if(!dreg_eql(reg,S6))
				{
					scamp7_dynamic_kernel_begin();
						MOV(reg,S6);
					scamp7_dynamic_kernel_end();
				}
			}


			void SCALE(dreg_t reg,int scaling_mag,bool scale_down)
			{
				if(!dreg_eql(reg,S6))
				{
					scamp7_dynamic_kernel_begin();
						MOV(S6,reg);
					scamp7_dynamic_kernel_end();
				}
				if(!scale_down)
				{
					for(unsigned char n = 0 ; n < scaling_mag ; n++)
					{
						STEP_SCALE_UP_S6(n);
					}
				}
				else
				{
					for(unsigned char n = 0 ; n < scaling_mag ; n++)
					{
						STEP_SCALE_DOWN_S6(n);
					}
				}
				if(!dreg_eql(reg,S6))
				{
					scamp7_dynamic_kernel_begin();
						MOV(reg,S6);
					scamp7_dynamic_kernel_end();
				}
			}



			int STEP_SCALE(dreg_t reg,int current_scaling_value, bool scale_DOWN)
			{
				if(!dreg_eql(reg,S6))
				{
					scamp7_dynamic_kernel_begin();
						MOV(S6,reg);
					scamp7_dynamic_kernel_end();
				}
				if(current_scaling_value > 0)
				{
					if(!scale_DOWN)
					{
						STEP_SCALE_UPX_S6(current_scaling_value);
						STEP_SCALE_UPY_S6(current_scaling_value);
						current_scaling_value = current_scaling_value + 1;
					}
					else
					{
						current_scaling_value = current_scaling_value - 1;
						STEP_SCALE_DOWNY_S6(current_scaling_value);
						STEP_SCALE_DOWNX_S6(current_scaling_value);
					}
				}
				else
				{
					if(current_scaling_value < 0)
					{
						if(!scale_DOWN)
						{
							current_scaling_value = current_scaling_value + 1;
							STEP_SCALE_UPY_S6(-current_scaling_value);
							STEP_SCALE_UPX_S6(-current_scaling_value);
						}
						else
						{
							STEP_SCALE_DOWNX_S6(-current_scaling_value);
							STEP_SCALE_DOWNY_S6(-current_scaling_value);
							current_scaling_value = current_scaling_value - 1;
						}
					}
					else
					{
						if(!scale_DOWN)
						{
							STEP_SCALE_UPX_S6(current_scaling_value);
							STEP_SCALE_UPY_S6(current_scaling_value);
							current_scaling_value = current_scaling_value + 1;
						}
						else
						{
							STEP_SCALE_DOWNX_S6(current_scaling_value);
							STEP_SCALE_DOWNY_S6(current_scaling_value);
							current_scaling_value = current_scaling_value - 1;
						}
					}
				}
				if(!dreg_eql(reg,S6))
				{
					scamp7_dynamic_kernel_begin();
						MOV(S6,reg);
					scamp7_dynamic_kernel_end();
				}
				return current_scaling_value;
			}



			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


			void HALF_SCALE(dreg_t reg) //USES S6 RF RM
			{
				scamp7_dynamic_kernel_begin();
					SET(RF);
					MOV(RM,reg);
				scamp7_dynamic_kernel_end();


				uint8_t x = 128;
				uint8_t y = 0;
				uint8_t w = 127;
				uint8_t h = 255;
				scamp7_load_rect(RW,y,x,y+h,x+w);
				scamp7_kernel_begin();
					MOV(RF,RW);
					CLR(RS,RW,RN,RE);
				scamp7_kernel_end();

				for(int n = 0; n < 128 ; n++)
				{
					scamp7_kernel_begin();
						SET(RW);
						WHERE(RM);
						DNEWS0(S6,FLAG);
						MOV(RM,S6);
						CLR(RW);

						SET(RE);
						DNEWS0(S6,RF);
						MOV(RF,S6);
						CLR(RE);
					scamp7_kernel_end();
				}

				x = 0;
				y = 0;
				w = 127;
				h = 255;
				scamp7_load_rect(RW,y,x,y+h,x+w);
				scamp7_kernel_begin();
					MOV(RF,RW);
				scamp7_kernel_end();
				for(int n = 0; n < 128 ; n++)
				{
					scamp7_kernel_begin();
						SET(RE);
						WHERE(RM);
						DNEWS0(S6,FLAG);
						MOV(RM,S6);
						CLR(RE);

						SET(RW);
						DNEWS0(S6,RF);
						MOV(RF,S6);
						CLR(RW);
					scamp7_kernel_end();
				}


				x = 0;
				y = 128;
				w = 255;
				h = 127;
				scamp7_load_rect(RS,y,x,y+h,x+w);
				scamp7_kernel_begin();
					MOV(RF,RS);
				scamp7_kernel_end();
				for(int n = 0; n < 128 ; n++)
				{
					scamp7_kernel_begin();
						SET(RS);
						WHERE(RM);
						DNEWS0(S6,FLAG);
						MOV(RM,S6);
						CLR(RS);

						SET(RN);
						DNEWS0(S6,RF);
						MOV(RF,S6);
						CLR(RN);
					scamp7_kernel_end();
				}

				x = 0;
				y = 0;
				w = 255;
				h = 127;
				scamp7_load_rect(RN,y,x,y+h,x+w);
				scamp7_kernel_begin();
					MOV(RF,RN);
				scamp7_kernel_end();
				for(int n = 0; n < 128 ; n++)
				{
					scamp7_kernel_begin();
						SET(RN);
						WHERE(RM);
						DNEWS0(S6,FLAG);
						MOV(RM,S6);
						CLR(RN);

						SET(RS);
						DNEWS0(S6,RF);
						MOV(RF,S6);
						CLR(RS);
					scamp7_kernel_end();
				}

				scamp7_dynamic_kernel_begin();
					MOV(reg,RM);
				scamp7_dynamic_kernel_end();

				scamp7_kernel_begin();
					ALL();
				scamp7_kernel_end();
			}
		}
	}
}
