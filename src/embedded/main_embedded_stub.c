/* main_embedded_stub.c - simple host stub that reads closed_loop.csv and writes logs/embedded_run_output.csv */
#include <stdio.h>
int main(void) {
    FILE *fin = fopen("closed_loop.csv","r");
    if (!fin) { printf("closed_loop.csv missing\n"); return 1; }
    FILE *fout = fopen("logs/embedded_run_output.csv","w");
    if (!fout) { printf("cannot open output\n"); fclose(fin); return 1; }
    char buf[256];
    if (!fgets(buf, sizeof(buf), fin)) return 1;
    fprintf(fout,"t,x,x_dot,theta,theta_dot,u_cmd\n");
    while (fgets(buf, sizeof(buf), fin)) {
        double t,x,x_dot,theta,theta_dot,u;
        if (sscanf(buf,"%lf,%lf,%lf,%lf,%lf,%lf",&t,&x,&x_dot,&theta,&theta_dot,&u) != 6) continue;
        fprintf(fout,"%lf,%lf,%lf,%lf,%lf,%lf\n", t,x,x_dot,theta,theta_dot,u);
    }
    fclose(fin); fclose(fout);
    printf("Stub run complete\n");
    return 0;
}