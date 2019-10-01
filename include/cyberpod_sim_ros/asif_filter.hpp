#ifndef ASIF_FILTER_H
#define ASIF_FILTER_H

#include "asif++.h"
#include <Eigen/Dense>

static const uint32_t nx = 4;
static const uint32_t nu = 1;
static const uint32_t npSS = 4;
static const uint32_t npBTSS = 4;

static const double lb[nu] = {-20.0};
static const double ub[nu] = {20.0};
static const double xBound[nx] = {3.0,3.0,M_PI/6,M_PI};
static const double xBoundBack[nx] = {3.0,3.0,M_PI/6,M_PI};

static const Eigen::VectorXd Kvec = 2.0*(Eigen::VectorXd(nx) << 44.7214, 44.6528, 150.1612, 37.6492).finished();

static const double *K = Kvec.data();
static const double Pv = 0.05;

static const double model[15] = {44.798,            //mb
                                 2.485,             //mw
                                 0.055936595310797, //Jw
                                 -0.02322718759275, //a2
                                 0.166845864363019, //c2
                                 3.604960049044268, //A2
                                 3.836289730154863, //B2
                                 1.069672194414735, //C2
                                 1.261650363363571, //K
                                 0.195,             //r
                                 0.5,               //L
                                 9.81,              //gGravity
                                 3.185188257847262, //FricCoeff
                                 1.0e-3,            //velEps
                                 1.225479467549329  //FricCoeff
                                 };

ASIF::ASIFimplicitTB *asif;

void safetySet(const double x[nx], double h[npSS], double Dh[npSS*nx])
{
	for(uint32_t i = 0; i<npSS*nx; i++)
	{
		Dh[i] = 0.0;
	}
	for(uint32_t i = 0; i<nx; i++)
	{
		h[i] = (xBound[i]*xBound[i])-(x[i]*x[i]);
		Dh[i*(nx+1)] = -2.0*x[i];
	}
}

void backupSet(const double x[nx], double h[1], double Dh[nx], double DDh[nx*nx])
{
	for(uint32_t i = 0; i<nx*nx; i++)
	{
		DDh[i] = 0.0;
	}

	h[0] = Pv*Pv;
	for(uint32_t i = 0; i<nx; i++)
	{
		h[0] -= (x[i]/xBound[i])*(x[i]/xBound[i]);
		Dh[i] = -2.0*x[i]/(xBound[i]*xBound[i]);
		DDh[i*(nx+1)] = -2.0/(xBound[i]*xBound[i]);
	}
}

void backupController(const double x[nx], double u[nu], double Du[nu*nx])
{
	double xTemp[nx] = {0.,0.,-0.1383244254,0.};
	for(uint32_t i=0; i<nx; i++)
	{
		xTemp[i]+=x[i];
	}
	ASIF::matrixVectorMultiply(K,nu,nx,
	                           xTemp,nx,
	                           u);
	memcpy(Du,K,sizeof(K));
}


void dynamics(const double X[nx], double f[nx], double g[nu*nx])
{
	double Fric;
	double f_tmp;
	double b_f_tmp;
	double c_f_tmp;
	double d_f_tmp;
	double e_f_tmp;
	double f_f_tmp;
	double g_f_tmp;
	double h_f_tmp;
	double i_f_tmp;
	double j_f_tmp;
	double k_f_tmp;
	double l_f_tmp;
	double f_tmp_tmp;
	double b_f_tmp_tmp;
	double m_f_tmp;
	double n_f_tmp;

/*  */
	Fric = X[1] - X[3] * model[9];
	Fric = model[12] * tanh(Fric / model[13]) + model[14] * Fric;
	f[0] = X[1];
	f_tmp = X[3] * X[3];
	b_f_tmp = model[4] * model[4];
	c_f_tmp = sin(X[2]);
	d_f_tmp = model[3] * model[3];
	e_f_tmp = model[0] * model[0];
	f_f_tmp = sin(2.0 * X[2]);
	g_f_tmp = 4.0 * d_f_tmp;
	h_f_tmp = model[9] * model[9];
	i_f_tmp = 4.0 * b_f_tmp;
	j_f_tmp = cos(2.0 * X[2]);
	k_f_tmp = cos(X[2]);
	l_f_tmp = d_f_tmp * e_f_tmp;
	f_tmp_tmp = l_f_tmp * h_f_tmp;
	b_f_tmp_tmp = b_f_tmp * e_f_tmp * h_f_tmp;
	i_f_tmp = 1.0 / ((((((((((4.0 * model[6] * model[2] + g_f_tmp * model[2] *
		model[0]) + i_f_tmp * model[2] * model[0]) + 2.0 * model[6] * model[0] *
	h_f_tmp) + f_tmp_tmp) + b_f_tmp_tmp) + 4.0 * model[6] * model[1] * h_f_tmp)
	+ g_f_tmp * model[0] * model[1] * h_f_tmp) + i_f_tmp *
	model[0] * model[1] * h_f_tmp) + (d_f_tmp + -b_f_tmp) *
	e_f_tmp * h_f_tmp * j_f_tmp) + 2.0 * model[3] * model[4] *
	e_f_tmp * h_f_tmp * f_f_tmp);
	m_f_tmp = 2.0 * d_f_tmp;
	f[1] = 0.5 * model[9] * ((((((((((-8.0 * model[6] * Fric + -8.0 * d_f_tmp *
		Fric * model[0]) + -8.0 * b_f_tmp * Fric * model[0]) + model[0] * (((-8.0 *
			model[4] * Fric + 4.0 * model[3] * model[6] * f_tmp) + 4.0 * pow
		(model[3], 3.0) * model[0] * f_tmp) + 4.0 * model[3] * b_f_tmp * model[0] *
		f_tmp) * model[9] * k_f_tmp) + -4.0 * model[3] * model[4] * model[11] *
	e_f_tmp * model[9] * j_f_tmp) + 8.0 * model[3] * Fric * model[0] * model[9] *
	c_f_tmp) + 4.0 * model[6] * model[4] * model[0] * f_tmp * model[9] * c_f_tmp)
	+ g_f_tmp * model[4] * e_f_tmp * f_tmp * model[9] * c_f_tmp) + 4.0 *
	pow(model[4], 3.0) * e_f_tmp * f_tmp * model[9] * c_f_tmp) + m_f_tmp
	* model[11] * e_f_tmp * model[9] * f_f_tmp) + -2.0 * b_f_tmp * model[11] *
	e_f_tmp * model[9] * f_f_tmp) * i_f_tmp;
	f[2] = X[3];
	g_f_tmp = 4.0 * model[4] * model[11];
	n_f_tmp = -(model[4] * model[4]) * e_f_tmp;
	f[3] = i_f_tmp * ((((((((((8.0 * Fric * model[2] + 4.0 * Fric * model[0] *
		h_f_tmp) + 8.0 * Fric * model[1] * h_f_tmp) + 2.0 * model[0] * (2.0 * model
	[4] * Fric * model[9] + model[3] * model[11] * (2.0 * model[2] + (model[0] +
		2.0 * model[1]) * h_f_tmp)) * k_f_tmp) + -2.0 * model[3] * model[4] *
	e_f_tmp * f_tmp * h_f_tmp * j_f_tmp) + g_f_tmp * model[2] * model[0] *
	c_f_tmp) + -4.0 * model[3] * Fric * model[0] * model[9] * c_f_tmp) + 2.0 *
	model[4] * model[11] * e_f_tmp * h_f_tmp * c_f_tmp) +
	g_f_tmp * model[0] * model[1] * h_f_tmp * c_f_tmp) +
	l_f_tmp * f_tmp * h_f_tmp * f_f_tmp) + n_f_tmp * f_tmp *
	h_f_tmp * f_f_tmp);
	g[0] = 0.0;
	Fric = 2.0 * b_f_tmp;
	f_tmp = model[4] * model[0] * model[9] * k_f_tmp;
	g_f_tmp = -model[3] * model[0] * model[9] * c_f_tmp;
	g[1] = 2.0 * model[8] * model[9] * ((((model[6] + d_f_tmp * model[0]) +
		b_f_tmp * model[0]) + f_tmp) + g_f_tmp) * (1.0 / (((((((((((2.0 * model[6] *
			model[2] + m_f_tmp * model[2] * model[0]) + Fric * model[2] * model[0]) +
		model[6] * model[0] * h_f_tmp) + f_tmp_tmp) + b_f_tmp_tmp) + 2.0 * model[6] *
		model[1] * h_f_tmp) + m_f_tmp * model[0] * model[1] * h_f_tmp) + Fric *
		model[0] * model[1] * h_f_tmp) + n_f_tmp * h_f_tmp * (k_f_tmp * k_f_tmp)) +
		-d_f_tmp * e_f_tmp * h_f_tmp * (c_f_tmp * c_f_tmp)) + model[3] * model[4] *
		e_f_tmp * h_f_tmp * f_f_tmp));
		g[2] = 0.0;
		g[3] = -4.0 * model[8] * ((((2.0 * model[2] + model[0] * h_f_tmp) + 2.0 *
			model[1] * h_f_tmp) + f_tmp) + g_f_tmp) * i_f_tmp;
}

void JfFun(const double in1[nx], double Jf[nx*nx])
{
	double t2;
	double t3;
	double t4;
	double t5;
	double t8;
	double t9;
	double t11;
	double t12;
	double t13;
	double t14;
	double t15;
	double t20;
	double t17;
	double t18;
	double t34;
	double t69;
	double t44_tmp_tmp;
	double t44_tmp;
	double t48;
	double t53_tmp;
	double t54;
	double t72_tmp_tmp;
	double t72_tmp;
	double t72;
	double t75_tmp;
	double b_t75_tmp;
	double t75;
	double t55;
	double t73_tmp;
	double b_t73_tmp;
	double c_t73_tmp;
	double t76;
	double t68;
	double t70;
	double Jf_tmp_tmp;
	double Jf_tmp;
	double b_Jf_tmp_tmp;
	double c_Jf_tmp_tmp;
	double b_Jf_tmp;
	double c_Jf_tmp;
	double d_Jf_tmp;
	double e_Jf_tmp;
	double f_Jf_tmp;
	double g_Jf_tmp;
	double h_Jf_tmp;
	double i_Jf_tmp;
	double j_Jf_tmp;
	double k_Jf_tmp;

/*     This function was generated by the Symbolic Math Toolbox version 8.3. */
/*     01-Oct-2019 00:14:47 */
	t2 = cos(in1[2]);
	t3 = sin(in1[2]);
	t4 = model[9] * model[14];
	t5 = model[9] * in1[3];
	t8 = model[0] * model[0];
	t9 = model[3] * model[3];
	t11 = model[4] * model[4];
	t12 = pow(model[4], 3.0);
	t13 = model[9] * model[9];
	t14 = in1[2] * 2.0;
	t15 = in1[3] * in1[3];
	t20 = 1.0 / model[13];
	t17 = cos(t14);
	t18 = sin(t14);
	t34 = t9 + -t11;
	t69 = t5 - in1[1];
	t14 = tanh(-t20 * t69);
	t44_tmp_tmp = model[3] * model[4];
	t44_tmp = t44_tmp_tmp * t8 * t13;
	t48 = model[12] * t14;
	t53_tmp = t8 * t13;
	t54 = model[12] * t20 * (t14 * t14 - 1.0);
	t72_tmp_tmp = model[14] * t69;
	t72_tmp = t48 - t72_tmp_tmp;
	t72 = model[3] * model[11] * (model[2] * 2.0 + t13 * (model[0] + model[1] * 2.0)) + model
	[4] * model[9] * t72_tmp * 2.0;
	t75_tmp = model[0] * model[2];
	b_t75_tmp = model[0] * model[1];
	t75 = 1.0 / ((((((((((model[2] * model[6] * 4.0 + t75_tmp * t9 * 4.0) + t75_tmp *
		t11 * 4.0) + model[0] * model[6] * t13 * 2.0) + model[1] * model
	[6] * t13 * 4.0) + t8 * t9 * t13) + t8 * t11 * t13) +
	b_t75_tmp * t9 * t13 * 4.0) + b_t75_tmp * t11 * t13 * 4.0) +
	t44_tmp * t18 * 2.0) + t53_tmp * t17 * t34);
	t55 = model[9] * t54;
	t14 = model[0] * model[3];
	t73_tmp = model[3] * model[6];
	b_t73_tmp = model[0] * pow(model[3], 3.0);
	c_t73_tmp = t14 * t11;
	t15 = ((t73_tmp * t15 * 4.0 + b_t73_tmp * t15 * 4.0) + c_t73_tmp * t15 * 4.0)
	+ -(model[4] * t72_tmp * 8.0);
	t76 = t75 * t75;
	t20 = model[14] + -t54;
	t68 = t4 + -t55;
	t69 = model[14] * 8.0 + -(t54 * 8.0);
	t70 = t4 * 8.0 + -(t55 * 8.0);
	Jf[0] = 0.0;
	Jf[1] = 0.0;
	Jf[2] = 0.0;
	Jf[3] = 0.0;
	Jf[4] = 1.0;
	t14 *= model[9];
	Jf_tmp_tmp = t14 * t3;
	Jf_tmp = Jf_tmp_tmp * t20;
	b_Jf_tmp_tmp = model[0] * model[4];
	c_Jf_tmp_tmp = b_Jf_tmp_tmp * model[9] * t2;
	b_Jf_tmp = c_Jf_tmp_tmp * t20;
	c_Jf_tmp = model[9] * t75;
	d_Jf_tmp = model[0] * t9;
	e_Jf_tmp = model[0] * t11;
	Jf[5] = c_Jf_tmp * ((((model[6] * t20 * 8.0 + d_Jf_tmp * t20 * 8.0) + e_Jf_tmp *
		t20 * 8.0) - Jf_tmp * 8.0) + b_Jf_tmp * 8.0) * -0.5;
	Jf[6] = 0.0;
	f_Jf_tmp = model[1] * t13;
	g_Jf_tmp = model[0] * t13;
	Jf[7] = t75 * ((((model[2] * t69 + g_Jf_tmp * (model[14] * 4.0 - t54 * 4.0)) +
		f_Jf_tmp * t69) - Jf_tmp * 4.0) + b_Jf_tmp * 4.0);
	Jf[8] = 0.0;
	Jf_tmp = model[9] * model[11] * t8;
	b_Jf_tmp = Jf_tmp * t9;
	Jf_tmp *= t11;
	h_Jf_tmp = b_Jf_tmp_tmp * model[6];
	b_Jf_tmp_tmp = t44_tmp_tmp * model[9];
	i_Jf_tmp = b_Jf_tmp_tmp * model[11] * t8;
	j_Jf_tmp = t14 * t2 * t72_tmp;
	t69 = t44_tmp * t17 * 4.0 - t53_tmp * t18 * t34 * 2.0;
	t54 = Jf_tmp_tmp * t72_tmp;
	t34 = t3 * t5 * t8 * t12;
	t44_tmp = model[0] * model[9] * t2;
	t53_tmp = h_Jf_tmp * t3 * t5;
	k_Jf_tmp = model[4] * t3 * t5 * t8 * t9;
	Jf[9] = c_Jf_tmp * (((((((-model[0] * model[9] * t3 * t15 + j_Jf_tmp * 8.0) +
		b_Jf_tmp * t17 * 4.0) - Jf_tmp * t17 * 4.0) + t2 * t5 * t8 * t12 * in1[3] *
	4.0) + h_Jf_tmp * t2 * t5 * in1[3] * 4.0) + model[4] * t2 * t5 * t8 * t9 *
	in1[3] * 4.0) + i_Jf_tmp * t18 * 8.0) / 2.0 - model[9] *
	t76 * t69 * ((((((((((model[6] * t72_tmp * -8.0 - d_Jf_tmp * t72_tmp * 8.0) -
		e_Jf_tmp * t72_tmp * 8.0) + t44_tmp * t15) + t54 * 8.0) + b_Jf_tmp * t18 *
	2.0) - Jf_tmp * t18 * 2.0) + t34 * in1[3] * 4.0) + t53_tmp
	* in1[3] * 4.0) + k_Jf_tmp * in1[3] * 4.0) - i_Jf_tmp * t17 *
	4.0) / 2.0;
	Jf[10] = 0.0;
	t14 = t5 * t5;
	Jf_tmp = t14 * t8;
	b_Jf_tmp = t48 * 8.0 - t72_tmp_tmp * 8.0;
	h_Jf_tmp = Jf_tmp * t9;
	i_Jf_tmp = t75_tmp * model[4] * model[11];
	t15 = model[4] * model[11];
	t20 = t44_tmp_tmp * t14 * t8;
	t14 = b_t75_tmp * model[4] * model[11];
	Jf[11] = t75 * (((((((model[0] * t3 * t72 * -2.0 + h_Jf_tmp * t17 * 2.0) -
		Jf_tmp * t11 * t17 * 2.0) - j_Jf_tmp * 4.0) + i_Jf_tmp *
	t2 * 4.0) + t15 * t2 * t8 * t13 * 2.0) + t20 * t18 * 4.0) +
	t14 * t2 * t13 * 4.0) - t76 * t69 * ((((((((((model[2] *
		b_Jf_tmp + f_Jf_tmp * b_Jf_tmp) + model[0] * t2 * t72 * 2.0) + g_Jf_tmp * (t48
	* 4.0 - t72_tmp_tmp * 4.0)) + h_Jf_tmp * t18) - t54 * 4.0) + i_Jf_tmp * t3 *
	4.0) + t15 * t3 * t8 * t13 * 2.0) + t5 * t8 * t11 * t18 * -t5) - t20 * t17 *
	2.0) + t14 * t3 * t13 * 4.0);
	Jf[12] = 0.0;
	Jf_tmp = Jf_tmp_tmp * t68;
	Jf[13] = c_Jf_tmp * (((((((model[6] * t68 * 8.0 + d_Jf_tmp * t68 * 8.0) +
		e_Jf_tmp * t68 * 8.0) + t34 * 8.0) + t44_tmp * (((model[4] * t68 * 8.0 +
			t73_tmp * in1[3] * 8.0) + b_t73_tmp * in1[3] * 8.0) + c_t73_tmp * in1[3] *
		8.0)) + t53_tmp * 8.0) - Jf_tmp * 8.0) + k_Jf_tmp * 8.0) / 2.0;
	Jf[14] = 1.0;
	b_Jf_tmp = model[9] * t5 * t8;
	Jf[15] = -t75 * (((((((model[2] * t70 + g_Jf_tmp * (t4 * 4.0 - t55 * 4.0)) +
		f_Jf_tmp * t70) - Jf_tmp * 4.0) + c_Jf_tmp_tmp * t68 *
	4.0) - b_Jf_tmp * t9 * t18 * 2.0) + b_Jf_tmp * t11 * t18 *
	2.0) + b_Jf_tmp_tmp * t5 * t8 * t17 * 4.0);
}


void dynamicsGradients(const double x[nx], double Df[nx*nx], double Dg[nx*nu*nx])
{
	double t2;
	double t3;
	double t4;
	double t5;
	double t6;
	double t7;
	double t8;
	double t11;
	double t13;
	double t15;
	double t18;
	double t30;
	double t31;
	double t28_tmp;
	double t28;
	double t29;
	double t36_tmp;
	double t36;
	double t37;
	double t45;
	double t44_tmp;
	double b_t44_tmp;
	double c_t44_tmp;
	double t44;
	double dv0[16];
	JfFun(x, Df);

/* JGFUN */
/*     JG = JGFUN(IN1,IN2) */
/*     This function was generated by the Symbolic Math Toolbox version 8.3. */
/*     01-Oct-2019 00:14:48 */
	t2 = cos(x[2]);
	t3 = sin(x[2]);
	t4 = model[0] * model[0];
	t5 = model[3] * model[3];
	t6 = model[4] * model[4];
	t7 = model[9] * model[9];
	t8 = x[2] * 2.0;
	t11 = cos(t8);
	t13 = sin(t8);
	t15 = model[0] * model[6] * t7;
	t8 = model[0] * model[4] * model[9];
	t18 = t8 * t2;
	t30 = t4 * t5 * t7;
	t31 = t4 * t6 * t7;
	t28_tmp = model[0] * model[3] * model[9];
	t28 = -(t28_tmp * t3);
	t29 = t5 + -t6;
	t36_tmp = model[3] * model[4] * t4 * t7;
	t36 = t36_tmp * t13;
	t37 = t28_tmp * t2 + t8 * t3;
	t8 = model[0] * model[2];
	t28_tmp = model[0] * model[1];
	t45 = model[2] * model[6];
	t44_tmp = t8 * t5;
	t8 *= t6;
	b_t44_tmp = model[1] * model[6] * t7;
	c_t44_tmp = t28_tmp * t5 * t7;
	t28_tmp = t28_tmp * t6 * t7;
	t4 *= t7;
	t44 = (((((((((t45 * 4.0 + t44_tmp * 4.0) + t8 * 4.0) + t15 * 2.0) + b_t44_tmp
		* 4.0) + t30) + t31) + c_t44_tmp * 4.0) + t28_tmp * 4.0) + t36 *
	2.0) + t4 * t11 * t29;
	t45 = ((((((((((t45 * 2.0 + t15) + t44_tmp * 2.0) + t8 * 2.0) + b_t44_tmp *
		2.0) + t30) + t31) + c_t44_tmp * 2.0) + t28_tmp * 2.0) + t36) +
	t4 * (t2 * t2) * -t6) + -(t3 * t3 * t30);
	memset(&dv0[0], 0, 9U * sizeof(double));
	t44_tmp = model[8] * model[9];
	t28_tmp = t2 * t3;
	t8 = t36_tmp * t11;
	dv0[9] = t44_tmp * t37 * -2.0 / t45 - t44_tmp / (t45 * t45) * ((t28_tmp * t30 *
		-2.0 + t28_tmp * t31 * 2.0) + t8 * 2.0) * ((((model[6] + t18) + t28) +
		model[0] * t5) + model[0] * t6) * 2.0;
		dv0[10] = 0.0;
		dv0[11] = model[8] * t37 * 4.0 / t44 - model[8] / (t44 * t44) * (t4 * t13 *
			t29 * 2.0 - t8 * 4.0) * ((((model[2] * 2.0 + t18) + t28) + model[0] * t7) +
			model[1] * t7 * 2.0) * 4.0;
			dv0[12] = 0.0;
			dv0[13] = 0.0;
			dv0[14] = 0.0;
			dv0[15] = 0.0;
			memcpy(&Dg[0], &dv0[0], sizeof(double) << 4);
}



#endif
