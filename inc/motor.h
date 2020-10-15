#ifndef MOTOR_H
#define MOTOR_H

struct InductionMachineSimulated
{
    double x[13]; ////////////////////////////////
    double rpm;
    double rpm_cmd;
    double rpm_deriv_cmd;
    double Tload;
    double Tem;

    double Lsigma;
    double rs;
    double rreq;
    double Lmu;
    double Lmu_inv;
    double alpha;

    double Js;
    double npp;
    double mu_m;
    double Ts;

    double ial;
    double ibe;

    double ual;
    double ube;
};

struct SynchronousMachineSimulated
{
    double rpm;
    double omg;
    double rpm_deriv_cmd;
    double Tload;
    double J;
    double Tem;
    double Ea;
    double Eb;

    double Bemf; //Back emf rms value @ 1000RPM
    double R;
    double Ld;
    double Lq;
    double KE;
    double L0;
    double L1;

    double Js;
    double npp; //pole pairs
    double mu_m;
    double Ts;

    double id;
    double iq;

    double phid;
    double phiq;

    double ial;
    double ibe;

    double ud;
    double uq;

    double ual;
    double ube;

    double theta_d;
};
extern struct SynchronousMachineSimulated ACM;

void Machine_init();
int machine_simulation(double ud, double uq);

#endif // !MOTOR_H
