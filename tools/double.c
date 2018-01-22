#include <stdio.h>
#include <assert.h>
#include <string.h>

int main(int argc, char *argv[])
{
  FILE *src = fopen(argv[1],"rb");
  char line[9999],m;
  int w,h,f,i;

  assert(src);
  fgets(line, 9999, src); assert(0 == strcmp(line, "P6\n") || 0 == strcmp(line, "P5\n"));
  m=line[1];
  fgets(line, 9999, src); assert(2 == sscanf(line, "%d %d", &w, &h));
  fgets(line, 9999, src); assert(1 == sscanf(line, "%d", &f));
  assert(3*w < 9999);
  assert(9 <= printf("P%c\n%d %d\n%d\n",m,w,2*h,f));

  f = (m=='5') ? 1 : 3;

  for(i=0; i<h; ++i)
  {
    assert(f*w == fread(line, 1, f*w, src));
    assert(f*w == fwrite(line, 1, f*w, stdout));
    assert(f*w == fwrite(line, 1, f*w, stdout));
  }

  h=ftell(src); fseek(src, 0, SEEK_END); assert(h == ftell(src));

  fclose(src);
  return 0;
}
