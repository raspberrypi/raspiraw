#include <stdio.h>
#include <assert.h>
#include <string.h>

int main(int argc, char *argv[])
{
  FILE *src = fopen(argv[1],"rb");
  char line[1999];
  int w,h,f,i;

  assert(src);
  fgets(line, 1999, src); assert(0 == strcmp(line, "P6\n"));
  fgets(line, 1999, src); assert(2 == sscanf(line, "%d %d", &w, &h));
  fgets(line, 1999, src); assert(1 == sscanf(line, "%d", &f));
  assert(3*w < 1999);
  assert(9 <= printf("P6\n%d %d\n%d\n",w,2*h,f));

  for(i=0; i<h; ++i)
  {
    assert(3*w == fread(line, 1, 3*w, src));
    assert(3*w == fwrite(line, 1, 3*w, stdout));
    assert(3*w == fwrite(line, 1, 3*w, stdout));
  }

  h=ftell(src); fseek(src, 0, SEEK_END); assert(h == ftell(src));

  fclose(src);
  return 0;
}
