#include <scamp7.hpp>

scamp7_kernel sobel_edge_S0([]{

    using namespace scamp7_kernel_api;

    // vertical edge
    movx(A,C,north);
    movx(B,C,south);
    add(A,A,B);
    add(A,A,C);
    add(A,A,C);

    movx(B,A,east);
    movx(A,A,west);

    sub(B,B,A);// B = B - A
    abs(D,B);// D is the vertical edge
    all();//BUG FIX

    // horizontal edge
    movx(A,C,east);
    movx(B,C,west);
    add(A,A,B);
    add(A,A,C);
    add(A,A,C);

    movx(B,A,south);
    movx(A,A,north);

    sub(B,B,A);// B = B - A
    abs(A,B);// A is the horizontal edge
    all();//BUG FIX

    add(A,A,D);// merge both result

    // digitize
    sub(A,A,E);
    where(A);
    	MOV(S4,FLAG);
    all();

//    // filter stand-alone points
//    DNEWS(S3,S4,east|west|north|south);
//    NOT(S2,S3);
//    CLR_IF(S4,S2);

    // merge result into S0
    MOV(S3,S0);
	OR(S0,S3,S4);

    res(A);

});


scamp7_kernel gain_x2_C([]{
	using namespace scamp7_kernel_api;
	where(C);
		mov(A,C);
		mov(B,C);
		add(C,A,B);
	all();
});


void acquire_edge_image(int gain,int edge_threshold, int edge_expansion,int HDR_iterations, int HDR_exposure_time)
{
	//first exposure
	scamp7_kernel_begin();
		all();
		get_image(C,F);
		CLR(S0);
		respix(F);// store reset level of PIX in F
	scamp7_kernel_end();

	// apply gain and get edge map
	scamp7_in(E,edge_threshold);

	scamp7_launch_kernel(sobel_edge_S0);

	for(int n = 0 ; n < gain ; n++)
	{
		scamp7_launch_kernel(gain_x2_C);
		scamp7_in(E,edge_threshold);
		scamp7_launch_kernel(sobel_edge_S0);
	}

	scamp7_kernel_begin();
		mov(D,C);
		respix(C);// store reset level of PIX in F
	scamp7_kernel_end();

	// short exposure iterations to deal with high light part
	for(int i=0;i<HDR_iterations;i++){
		vs_wait(HDR_exposure_time);

		scamp7_kernel_begin();
			getpix(C,F);
		scamp7_kernel_end();

		scamp7_in(E,edge_threshold);
		scamp7_launch_kernel(sobel_edge_S0);

		scamp7_kernel_begin();
			mov(E,C);
			add(C,C,D);
		scamp7_kernel_end();

		scamp7_in(E,edge_threshold);
		scamp7_launch_kernel(sobel_edge_S0);

		scamp7_kernel_begin();
			mov(C,E);
		scamp7_kernel_end();
	}

	for(int n = 0 ; n < edge_expansion ; n++)
	{
		scamp7_kernel_begin();
			DNEWS(S6,S0,east | west | south | north);
			MOV(RS,S0);
			OR(S0,RS,S6);
		scamp7_kernel_end();
	}
}
