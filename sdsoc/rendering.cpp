/*===============================================================*/
/*                                                               */
/*                        rendering.cpp                          */
/*                                                               */
/*                 C++ kernel for 3D Rendering                   */
/*                                                               */
/*===============================================================*/

#include "../host/typedefs.h"

/*======================UTILITY FUNCTIONS========================*/


// Determine whether three vertices of a trianlgLe
// (x0,y0) (x1,y1) (x2,y2) are in clockwise order by Pineda algorithm
// if so, return a number > 0
// else if three points are in line, return a number == 0
// else in counterclockwise order, return a number < 0
int check_clockwise( Triangle_2D triangle_2d )
{
  int cw;

  cw = (triangle_2d.x2 - triangle_2d.x0) * (triangle_2d.y1 - triangle_2d.y0)
       - (triangle_2d.y2 - triangle_2d.y0) * (triangle_2d.x1 - triangle_2d.x0);

  return cw;

}

// swap (x0, y0) (x1, y1) of a Triangle_2D
void clockwise_vertices( Triangle_2D *triangle_2d )
{

  bit8 tmp_x, tmp_y;

  tmp_x = triangle_2d->x0;
  tmp_y = triangle_2d->y0;

  triangle_2d->x0 = triangle_2d->x1;
  triangle_2d->y0 = triangle_2d->y1;

  triangle_2d->x1 = tmp_x;
  triangle_2d->y1 = tmp_y;

}


// Given a pixel, determine whether it is inside the triangle
// by Pineda algorithm
// if so, return true
// else, return false
bit1 pixel_in_triangle( bit8 x, bit8 y, Triangle_2D triangle_2d )
{

  int pi0, pi1, pi2;

  pi0 = (x - triangle_2d.x0) * (triangle_2d.y1 - triangle_2d.y0) - (y - triangle_2d.y0) * (triangle_2d.x1 - triangle_2d.x0);
  pi1 = (x - triangle_2d.x1) * (triangle_2d.y2 - triangle_2d.y1) - (y - triangle_2d.y1) * (triangle_2d.x2 - triangle_2d.x1);
  pi2 = (x - triangle_2d.x2) * (triangle_2d.y0 - triangle_2d.y2) - (y - triangle_2d.y2) * (triangle_2d.x0 - triangle_2d.x2);

  return (pi0 >= 0 && pi1 >= 0 && pi2 >= 0);
}

// find the min from 3 integers
bit8 find_min( bit8 in0, bit8 in1, bit8 in2 )
{
  if (in0 < in1)
  {
    if (in0 < in2)
      return in0;
    else 
      return in2;
  }
  else 
  {
    if (in1 < in2) 
      return in1;
    else 
      return in2;
  }
}


// find the max from 3 integers
bit8 find_max( bit8 in0, bit8 in1, bit8 in2 )
{
  if (in0 > in1)
  {
    if (in0 > in2)
      return in0;
    else 
      return in2;
  }
  else 
  {
    if (in1 > in2) 
      return in1;
    else 
      return in2;
  }
}

/*======================PROCESSING STAGES========================*/

// project a 3D triangle to a 2D triangle
void projection ( int data_in[10], int data_out[8])
{
#pragma HLS INTERFACE ap_hs port=data_out
#pragma HLS INTERFACE ap_hs port=data_in
  //#pragma HLS INLINE off
  // Setting camera to (0,0,-1), the canvas at z=0 plane
  // The 3D model lies in z>0 space
  // The coordinate on canvas is proportional to the corresponding coordinate 
  // on space

  Triangle_3D triangle_3d;
  int angle;
  Triangle_2D triangle_2d;

  int counter;
  counter = data_in[0];
  angle = data_in[1];
  triangle_3d.x0 = (bit8)data_in[2];
  triangle_3d.y0 = (bit8)data_in[3];
  triangle_3d.z0 = (bit8)data_in[4];
  triangle_3d.x1 = (bit8)data_in[5];
  triangle_3d.y1 = (bit8)data_in[6];
  triangle_3d.z1 = (bit8)data_in[7];
  triangle_3d.x2 = (bit8)data_in[8];
  triangle_3d.y2 = (bit8)data_in[9];
  triangle_3d.z2 = (bit8)data_in[10];


  if(angle == 0)
  {
    triangle_2d.x0 = triangle_3d.x0;
    triangle_2d.y0 = triangle_3d.y0;
    triangle_2d.x1 = triangle_3d.x1;
    triangle_2d.y1 = triangle_3d.y1;
    triangle_2d.x2 = triangle_3d.x2;
    triangle_2d.y2 = triangle_3d.y2;
    triangle_2d.z  = triangle_3d.z0 / 3 + triangle_3d.z1 / 3 + triangle_3d.z2 / 3;
  }

  else if(angle == 1)
  {
    triangle_2d.x0 = triangle_3d.x0;
    triangle_2d.y0 = triangle_3d.z0;
    triangle_2d.x1 = triangle_3d.x1;
    triangle_2d.y1 = triangle_3d.z1;
    triangle_2d.x2 = triangle_3d.x2;
    triangle_2d.y2 = triangle_3d.z2;
    triangle_2d.z  = triangle_3d.y0 / 3 + triangle_3d.y1 / 3 + triangle_3d.y2 / 3;
  }
      
  else if(angle == 2)
  {
    triangle_2d.x0 = triangle_3d.z0;
    triangle_2d.y0 = triangle_3d.y0;
    triangle_2d.x1 = triangle_3d.z1;
    triangle_2d.y1 = triangle_3d.y1;
    triangle_2d.x2 = triangle_3d.z2;
    triangle_2d.y2 = triangle_3d.y2;
    triangle_2d.z  = triangle_3d.x0 / 3 + triangle_3d.x1 / 3 + triangle_3d.x2 / 3;
  }
  data_out[0] = counter;
  data_out[1] = triangle_2d.x0;
  data_out[2] = triangle_2d.y0;
  data_out[3] = triangle_2d.x1;
  data_out[4] = triangle_2d.y1;
  data_out[5] = triangle_2d.x2;
  data_out[6] = triangle_2d.y2;
  data_out[7] = triangle_2d.z;

}

// calculate bounding box for a 2D triangle
void rasterization1 ( int data_in[8], int data_out[15])
{
#pragma HLS INTERFACE ap_hs port=data_out
#pragma HLS INTERFACE ap_hs port=data_in
  Triangle_2D triangle_2d;
  bit8 max_min[5];
  bit16 max_index;
  Triangle_2D triangle_2d_same;
  int flag;
  int counter;
  counter = data_in[0];
  triangle_2d.x0 = data_in[1];
  triangle_2d.y0 = data_in[2];
  triangle_2d.x1 = data_in[3];
  triangle_2d.y1 = data_in[4];
  triangle_2d.x2 = data_in[5];
  triangle_2d.y2 = data_in[6];
  triangle_2d.z  = data_in[7];


  //#pragma HLS INLINE off
  // clockwise the vertices of input 2d triangle
  if ( check_clockwise( triangle_2d ) == 0 ){

	  data_out[0] = counter;
	  data_out[1] = 1;
	  data_out[2] = 0;
	  data_out[3] = 0;
	  data_out[4] = 0;
	  data_out[5] = 0;
	  data_out[6] = 0;
	  data_out[7] = 0;
	  data_out[8] = 0;
	  data_out[9] = 0;
	  data_out[10] = 0;
	  data_out[11] = 0;
	  data_out[12] = 0;
	  data_out[13] = 0;
	  data_out[14] = 0;
	  return;
  }


  if ( check_clockwise( triangle_2d ) < 0 )
    clockwise_vertices( &triangle_2d );

  // copy the same 2D triangle
  triangle_2d_same.x0 = triangle_2d.x0;
  triangle_2d_same.y0 = triangle_2d.y0;
  triangle_2d_same.x1 = triangle_2d.x1;
  triangle_2d_same.y1 = triangle_2d.y1;
  triangle_2d_same.x2 = triangle_2d.x2;
  triangle_2d_same.y2 = triangle_2d.y2;
  triangle_2d_same.z  = triangle_2d.z ;

  // find the rectangle bounds of 2D triangles
  max_min[0] = find_min( triangle_2d.x0, triangle_2d.x1, triangle_2d.x2 );
  max_min[1] = find_max( triangle_2d.x0, triangle_2d.x1, triangle_2d.x2 );
  max_min[2] = find_min( triangle_2d.y0, triangle_2d.y1, triangle_2d.y2 );
  max_min[3] = find_max( triangle_2d.y0, triangle_2d.y1, triangle_2d.y2 );
  max_min[4] = max_min[1] - max_min[0];

  // calculate index for searching pixels
  max_index = (max_min[1] - max_min[0]) * (max_min[3] - max_min[2]);
  data_out[0] = counter;
  data_out[1] = 0;
  data_out[2] = max_index;
  data_out[3] = max_min[0];
  data_out[4] = max_min[1];
  data_out[5] = max_min[2];
  data_out[6] = max_min[3];
  data_out[7] = max_min[4];
  data_out[8] = triangle_2d_same.x0;
  data_out[9] = triangle_2d_same.y0;
  data_out[10] = triangle_2d_same.x1;
  data_out[11] = triangle_2d_same.y1;
  data_out[12] = triangle_2d_same.x2;
  data_out[13] = triangle_2d_same.y2;
  data_out[14] = triangle_2d_same.z;


  return;
}

// find pixels in the triangles from the bounding box
void rasterization2 (int data_in[15], int data_out[2002])
{
#pragma HLS INTERFACE ap_hs port=data_in
#pragma HLS INTERFACE ap_hs port=data_out
	bit2 flag;
	bit8 max_min_0, max_min_1, max_min_2, max_min_3, max_min_4;
	bit16 max_index;
	Triangle_2D triangle_2d_same;
	int data_in_tmp[15];
	CandidatePixel fragment2[500];
	for(int i_ylx=0; i_ylx<15; i_ylx++)
	{
		data_in_tmp[i_ylx] = data_in[i_ylx];
	}
	int counter;
	counter = data_in_tmp[0];
	flag = data_in_tmp[1];
	max_index = data_in_tmp[2];
    max_min_0 = data_in_tmp[3];
    max_min_1 = data_in_tmp[4];
    max_min_2 = data_in_tmp[5];
    max_min_3 = data_in_tmp[6];
    max_min_4 = data_in_tmp[7];
    triangle_2d_same.x0 = data_in_tmp[8];
    triangle_2d_same.y0 = data_in_tmp[9];
    triangle_2d_same.x1 = data_in_tmp[10];
    triangle_2d_same.y1 = data_in_tmp[11];
    triangle_2d_same.x2 = data_in_tmp[12];
    triangle_2d_same.y2 = data_in_tmp[13];
    triangle_2d_same.z = data_in_tmp[14];


  #pragma HLS INLINE off
  // clockwise the vertices of input 2d triangle
  if ( flag )
  {
	  data_out[0] = counter;
	  for (int i_ylx=0; i_ylx<500; i_ylx++){
	 	  data_out[4*i_ylx+1] = fragment2[i_ylx].x;
	 	  data_out[4*i_ylx+2] = fragment2[i_ylx].y;
	 	  data_out[4*i_ylx+3] = fragment2[i_ylx].z;
	 	  data_out[4*i_ylx+4] = fragment2[i_ylx].color;
	  }
	  data_out[2001] = 0;
	  return;
  }

  bit8 color = 100;
  bit16 i = 0;
  RAST2: for ( bit16 k = 0; k < max_index; k++ )
  {
    #pragma HLS PIPELINE II=1
    bit8 x = max_min_0 + k%max_min_4;
    bit8 y = max_min_2 + k/max_min_4;

    if( pixel_in_triangle( x, y, triangle_2d_same ) )
    {
      fragment2[i].x = x;
      fragment2[i].y = y;
      fragment2[i].z = triangle_2d_same.z;
      fragment2[i].color = color;
      i++;
    }
  }

  data_out[0] = counter;
  for (int i_ylx=0; i_ylx<500; i_ylx++){
 	  data_out[4*i_ylx+1] = fragment2[i_ylx].x;
 	  data_out[4*i_ylx+2] = fragment2[i_ylx].y;
 	  data_out[4*i_ylx+3] = fragment2[i_ylx].z;
 	  data_out[4*i_ylx+4] = fragment2[i_ylx].color;
   }

  data_out[2001] = i;
  return;
}

// filter hidden pixels
void zculling (int data_in[2002], int data_out[1502])
{
#pragma HLS INTERFACE ap_hs port=data_out
#pragma HLS INTERFACE ap_hs port=data_in
	CandidatePixel fragments[500];
	bit16 size;
	Pixel pixels[500];
	bit16 counter;
	counter = data_in[0];

    for (int i_ylx=0; i_ylx<500; i_ylx++){
  	  fragments[i_ylx].x = data_in[4*i_ylx+1];
  	  fragments[i_ylx].y = data_in[4*i_ylx+2];
  	  fragments[i_ylx].z = data_in[4*i_ylx+3];
  	  fragments[i_ylx].color = data_in[4*i_ylx+4];
    }
    size = data_in[2001];

  // initilize the z-buffer in rendering first triangle for an image
  static bit8 z_buffer[MAX_X][MAX_Y];
  if (counter == 0)
  {
    ZCULLING_INIT_ROW: for ( bit16 i = 0; i < MAX_X; i++)
    {
      #pragma HLS PIPELINE II=1
      ZCULLING_INIT_COL: for ( bit16 j = 0; j < MAX_Y; j++)
      {
        z_buffer[i][j] = 255;
      }
    }
  }

  // pixel counter
  bit16 pixel_cntr = 0;
  
  // update z-buffer and pixels
  ZCULLING: for ( bit16 n = 0; n < size; n++ ) 
  {
    #pragma HLS PIPELINE II=1
    if( fragments[n].z < z_buffer[fragments[n].y][fragments[n].x] )
    {
      pixels[pixel_cntr].x     = fragments[n].x;
      pixels[pixel_cntr].y     = fragments[n].y;
      pixels[pixel_cntr].color = fragments[n].color;
      pixel_cntr++;
      z_buffer[fragments[n].y][fragments[n].x] = fragments[n].z;
    }
  }

  data_out[0] = counter;
  for (int i_ylx=0; i_ylx<500; i_ylx++){
	  data_out[i_ylx*3+1] = pixels[i_ylx].x;
	  data_out[i_ylx*3+2] = pixels[i_ylx].y;
	  data_out[i_ylx*3+3] = pixels[i_ylx].color;
  }
  data_out[1501] = pixel_cntr;
}

// color the frame buffer
void coloringFB(int data_in[1502],  bit8 frame_buffer[MAX_X][MAX_Y])
{
#pragma HLS INTERFACE ap_hs port=data_in
#pragma HLS INTERFACE ap_hs port=frame_buffer
	Pixel pixels[500];
	bit16 size_pixels;
	bit16 counter;
	counter = data_in[0];
	for (int i_ylx=0; i_ylx<500; i_ylx++){
	  pixels[i_ylx].x =  data_in[i_ylx*3+1];
	  pixels[i_ylx].y = data_in[i_ylx*3+2];
	  pixels[i_ylx].color = data_in[i_ylx*3+3];
	}
	size_pixels = data_in[1501];

  if ( counter == 0 )
  {
    // initilize the framebuffer for a new image
    COLORING_FB_INIT_ROW: for ( bit16 i = 0; i < MAX_X; i++)
    {
      #pragma HLS PIPELINE II=1
      COLORING_FB_INIT_COL: for ( bit16 j = 0; j < MAX_Y; j++)
        frame_buffer[i][j] = 0;
    }
  }

  // update the framebuffer
  COLORING_FB: for ( bit16 i = 0; i < size_pixels; i++)
  {
    #pragma HLS PIPELINE II=1
    frame_buffer[ pixels[i].x ][ pixels[i].y ] = pixels[i].color;
  }

}

// stream out the frame buffer
void output_FB(bit8 frame_buffer[MAX_X][MAX_Y], bit32 output[NUM_FB])
{
#pragma HLS INTERFACE ap_hs port=output
#pragma HLS INTERFACE ap_hs port=frame_buffer
  #pragma HLS INLINE
  bit32 out_FB = 0;
  OUTPUT_FB_ROW: for ( bit16 i = 0; i < MAX_X; i++)
  {
    #pragma HLS PIPELINE II=1
    OUTPUT_FB_COL: for ( bit16 j = 0; j < MAX_Y; j = j + 4)
    {
      out_FB( 7,  0) = frame_buffer[i][j + 0];
      out_FB(15,  8) = frame_buffer[i][j + 1];
      out_FB(23, 16) = frame_buffer[i][j + 2];
      out_FB(31, 24) = frame_buffer[i][j + 3];
      output[i * MAX_Y / 4 + j / 4] = out_FB;
    }
  }
}


/*========================TOP FUNCTION===========================*/
void rendering( bit32 input[3*NUM_3D_TRI], bit32 output[NUM_FB])
{
  // local variables
  Triangle_3D triangle_3ds;
  Triangle_2D triangle_2ds;
  Triangle_2D triangle_2ds_same;

  bit16 size_fragment;
  CandidatePixel fragment[500];

  bit16 size_pixels;
  Pixel pixels[500];

  bit8 frame_buffer[MAX_X][MAX_Y];
  bit2 angle = 0;

  bit8 max_min[5];
  bit16 max_index[1];
  bit2 flag;

  // processing NUM_3D_TRI 3D triangles
  TRIANGLES: for (bit16 i = 0; i < NUM_3D_TRI; i++)
  {
    bit32 input_lo  = input[3*i];
    bit32 input_mi  = input[3*i+1];
    bit32 input_hi  = input[3*i+2];

    triangle_3ds.x0 = input_lo( 7,  0);
    triangle_3ds.y0 = input_lo(15,  8);
    triangle_3ds.z0 = input_lo(23, 16);
    triangle_3ds.x1 = input_lo(31, 24);
    triangle_3ds.y1 = input_mi( 7,  0);
    triangle_3ds.z1 = input_mi(15,  8);
    triangle_3ds.x2 = input_mi(23, 16);
    triangle_3ds.y2 = input_mi(31, 24);
    triangle_3ds.z2 = input_hi( 7,  0);


    #ifdef USE_DATAFLOW
      #pragma HLS dataflow
    #endif

    // five stages for processing each 3D triangle
    int data_in_pro[11];
    int data_tmp_1[8];
    int data_tmp_2[15];
    int data_tmp_3[2002];
    int data_tmp_4[1502];

    data_in_pro[0] = i;
    data_in_pro[1] = angle;
    data_in_pro[2] = triangle_3ds.x0;
    data_in_pro[3] = triangle_3ds.y0;
    data_in_pro[4] = triangle_3ds.z0;
    data_in_pro[5] = triangle_3ds.x1;
    data_in_pro[6] = triangle_3ds.y1;
    data_in_pro[7] = triangle_3ds.z1;
    data_in_pro[8] = triangle_3ds.x2;
    data_in_pro[9] = triangle_3ds.y2;
    data_in_pro[10] = triangle_3ds.z2;

    projection(data_in_pro, data_tmp_1);
    rasterization1( data_tmp_1, data_tmp_2);
    rasterization2(data_tmp_2, data_tmp_3);
    zculling(data_tmp_3, data_tmp_4);
    coloringFB (data_tmp_4, frame_buffer);
  }

  // output values: frame buffer
  output_FB(frame_buffer,output);
}
