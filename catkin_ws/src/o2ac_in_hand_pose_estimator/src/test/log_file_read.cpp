#include <cstdio>
#include <iostream>
#include <cstring>

int main(int argc, char **argv)
{  
  FILE *pose_file=fopen(argv[1], "r");
  FILE *log_file=fopen(argv[2], "r");
  char message[999];
  fgets(message, 999, pose_file);
  fgets(message, 999, log_file);
  while(1){
    unsigned long long time;
    fscanf(log_file, "%llu,\"data: \"\"%[^\"]\"\"\"\n", &time, message);
    //printf("%s\n",message);
    if(strncmp(message, "end",3) == 0){
      break;
    }
    else if(strncmp(message, "touching_table",10) == 0){
      unsigned long long pose_time;
      while(1){
	fscanf(pose_file,"%llu,",&pose_time);
	if(pose_time >= time){
	  break;
	}
	else{
	  fgets(message, 999, pose_file);
	}
      }
      double x, y, z, qa, qb, qc, qd;
      fscanf(pose_file, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",&x,&y,&z,&qa,&qb,&qc,&qd);
      printf("%lf %lf %lf %lf %lf %lf %lf\n",x,y,z,qa,qb,qc,qd);
    }
  }
  fclose(pose_file);
  fclose(log_file);
}
