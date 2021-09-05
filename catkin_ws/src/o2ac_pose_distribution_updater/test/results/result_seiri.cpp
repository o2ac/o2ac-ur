#include <cstdio>
#include <string>

int main()
{
  std::string obj_names[13]={"base",
			   "panel_motor",
			   "panel_bearing",
			   "motor",
			   "motor_pulley",
			   "bearing",
			   "shaft",
			   "end_cap",
			   "bearing_spacer",
			   "output_pulley",
			   "idler_spacer",
			   "idler_pulley",
			   "idler_pin"
  };
  std::string type_patterns[11]={
			       "11111",
			       "01111",
			       "10111",
			       "00111",
			       "11110",
			       "01110",
			       "10110",
			       "00110",
			       "11000",
			       "01000",
			       "10000"
  };
  double score[13][11][4], average[11][4]={0.};
  for(int i=0;i<13;i++){
    for(int j=0;j<11;j++){
      std::string file_name=obj_names[i]+"-"+type_patterns[j]+".txt";
      FILE* file=fopen(file_name.c_str(), "r");
      char c;
      while((c=fgetc(file))!='\n');
      for(int k=0;k<4;k++){
	fscanf(file, "%lf", score[i][j]+k);
	printf("%10.10lf ",score[i][j][k]);
	average[j][k]+=score[i][j][k]/13.;
      }
      fclose(file);
      putchar('\n');
    }
    putchar('\n');
  }
  for(int j=0;j<11;j++){
    for(int k=0;k<4;k++){
      printf("%10.10lf ",average[j][k]);
    }
    putchar('\n');
  }
  return 0;
}
