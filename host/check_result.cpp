/*===============================================================*/
/*                                                               */
/*                       check_result.cpp                        */
/*                                                               */
/*      Software evaluation of training and test error rate      */
/*                                                               */
/*===============================================================*/

#include <cstdio>
#include "typedefs.h"
#include "RightData.h"
#ifndef SW
void check_results(bit32* output)
#else
void check_results(bit8 output[MAX_X][MAX_Y])
#endif
{
  int data_tmp[256][256];

  #ifndef SW
    bit8 frame_buffer_print[MAX_X][MAX_Y];
  
    // read result from the 32-bit output buffer
    for (int i = 0, j = 0, n = 0; n < NUM_FB; n ++ )
    {
      bit32 temp = output[n];
      frame_buffer_print[i][j++] = temp(7,0);
      frame_buffer_print[i][j++] = temp(15,8);
      frame_buffer_print[i][j++] = temp(23,16);
      frame_buffer_print[i][j++] = temp(31,24);
      if(j == MAX_Y)
      {
        i++;
        j = 0;
      }
    }
  #endif

  // print result
  printf("Image After Rendering: \n");
  for (int j = MAX_X - 1; j >= 0; j -- )
  {
    for (int i = 0; i < MAX_Y; i ++ )
    {
      int pix;
      #ifndef SW
        pix = frame_buffer_print[i][j].to_int();
      #else
        pix = output[i][j];
      #endif
      if (pix){
    	data_tmp[255-j][i] = 1;
    	//printf("1");
      } else {
    	data_tmp[255-j][i] = 0;
        //printf("0");
      }
    }
    printf("\n");
  }

  for (int j = MAX_X - 1; j >= 0; j -- )
  {
    for (int i = 0; i < MAX_Y; i ++ )
    {
      if(data_tmp[j][i] != RightData[j][i]){
    	  printf("++++++++++++++++++\nSomething is wrong!\n");
    	  return;
      }

    }

  }
  printf("Everything is OK!\n\r");

}
