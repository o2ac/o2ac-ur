#include <tuple>
#include <map>
#include <vector>
#include <cstring>

void read_stl(FILE *in, 
	      std::vector<std::vector<double> > &points,
	      std::vector<std::vector<int> > &tris)
{
  char s[7];
  fgets(s, 6, in);
  if(strcmp(s, "solid") == 0){
    // ASCII mode
    char c;
    while((c=fgetc(in))!='\n');
    using tuple = std::tuple<std::string, std::string, std::string>;
    std::map<tuple,int> index;
    while(1){
      char next[10];
      fscanf(in, "%s", next);
      if(strcmp(next, "endsolid") == 0)
	break;
      while((c=fgetc(in))!='\n');
      while((c=fgetc(in))!='\n');
      std::vector<int> tri(3);
      for(int i=0;i<3;i++){
	char str_x[99], str_y[99], str_z[99];
	fscanf(in, "%s%s%s%s",next, str_x, str_y, str_z);
	tuple t = std::make_tuple(std::string(str_x),std::string(str_y),std::string(str_z));
	if(index.find(t) == index.end()){
	  index[t] = points.size();
	  std::vector<double> vec{atof(str_x), atof(str_y), atof(str_z)};
	  points.push_back(vec);
	}
	tri[i]=index[t];
      }
      tris.push_back(tri);
      while((c=fgetc(in))!='\n');
      while((c=fgetc(in))!='\n');
      while((c=fgetc(in))!='\n');
    }
  }
  else{
    // Binary mode
    fseek(in, 0, SEEK_SET);
    char comment[80];
    fread(comment, 1, 80, in);
    unsigned int num;
    fread(&num, 4, 1, in);
    tris=std::vector<std::vector<int> >(num, std::vector<int>(3));
    using tuple = std::tuple<float, float, float>;
    std::map<tuple,int> index;
    for(int tri=0;tri<num;tri++){
      float norm[3], v[3][3];
      fread(norm, 4, 3, in);
      for(int i=0;i<3;i++){
	fread(v[i], 4, 3, in);
	tuple t = std::make_tuple(v[i][0], v[i][1], v[i][2]);
	if(index.find(t) == index.end()){
	  index[t] = points.size();
	  std::vector<double> vec{v[i][0], v[i][1], v[i][2]};
	  points.push_back(vec);
	}
	tris[tri][i]=index[t];
      }
      unsigned short int unused;
      fread(&unused, 2, 1, in);
    }
  }
}
