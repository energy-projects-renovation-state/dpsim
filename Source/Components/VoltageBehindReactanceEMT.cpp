#include "VoltageBehindReactanceEMT.h"

using namespace DPsim;

VoltageBehindReactanceEMT::VoltageBehindReactanceEMT(std::string name, int node1, int node2, int node3,
	Real nomPower, Real nomVolt, Real nomFreq, int poleNumber, Real nomFieldCur,
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real inertia) {

	this->mNode1 = node1 - 1;
	this->mNode2 = node2 - 1;
	this->mNode3 = node3 - 1;

	mNomPower = nomPower;
	mNomVolt = nomVolt;
	mNomFreq = nomFreq;
	mPoleNumber = poleNumber;
	mNomFieldCur = nomFieldCur;

	// base stator values
	mBase_V_RMS = mNomVolt / sqrt(3);
	mBase_v = mBase_V_RMS * sqrt(2);
	mBase_I_RMS = mNomPower / (3 * mBase_V_RMS);
	mBase_i = mBase_I_RMS * sqrt(2);
	mBase_Z = mBase_v / mBase_i;
	mBase_OmElec = 2 * DPS_PI * mNomFreq;
	mBase_OmMech = mBase_OmElec / (mPoleNumber / 2);
	mBase_L = mBase_Z / mBase_OmElec;
	mBase_Psi = mBase_L * mBase_i;
	mBase_T = mNomPower / mBase_OmMech;

	// steady state per unit initial value
	initWithPerUnitParam(Rs, Ll, Lmd, Lmd0, Lmq, Lmq0, Rfd, Llfd, Rkd, Llkd, Rkq1, Llkq1, Rkq2, Llkq2, inertia);
	
}
void VoltageBehindReactanceEMT::initWithPerUnitParam(
	Real Rs, Real Ll, Real Lmd, Real Lmd0, Real Lmq, Real Lmq0,
	Real Rfd, Real Llfd, Real Rkd, Real Llkd,
	Real Rkq1, Real Llkq1, Real Rkq2, Real Llkq2,
	Real H) {

	// base rotor values
	mBase_ifd = Lmd * mNomFieldCur;
	mBase_vfd = mNomPower / mBase_ifd;
	mBase_Zfd = mBase_vfd / mBase_ifd;
	mBase_Lfd = mBase_Zfd / mBase_OmElec;

	mRs = Rs;
	mLl = Ll;
	mLmd = Lmd;
	mLmd0 = Lmd0;
	mLmq = Lmq;
	mLmq0 = Lmq0;
	mRfd = Rfd;
	mLlfd = Llfd;
	mRkd = Rkd;
	mLlkd = Llkd;
	mRkq1 = Rkq1;
	mLlkq1 = Llkq1;
	mRkq2 = Rkq2;
	mLlkq2 = Llkq2;
	mH = H;
		

	//Dynamic mutual inductances
	mDLmd = 1. / (1. / mLmd + 1. / mLlfd + 1. / mLlkd);
	mDLmq = 1. / (1. / mLmq + 1. / mLlkq1 + 1. / mLlkq2);

	mLa = (mDLmq + mDLmd) / 3.;
	mLb = (mDLmd - mDLmq) / 3.;

	b11 = (mRkq1 / mLlkq1)*(mDLmq / mLlkq1 - 1);
	b12 = mRkq1*mDLmq / (mLlkq1*mLlkq2);
	b13 = mRkq1*mDLmq / mLlkq1;
	b21 = mRkq2*mDLmq / (mLlkq1*mLlkq2);
	b22 = (mRkq2 / mLlkq2)*(mDLmq / mLlkq2 - 1);
	b23 = mRkq2*mDLmq / mLlkq2;
	b31 = (mRfd / mLlfd)*(mDLmd / mLlfd - 1);
	b32 = mRfd*mDLmd / (mLlfd*mLlkd);
	b33 = mRfd*mDLmd / mLlfd;
	b41 = mRkd*mDLmd / (mLlfd*mLlkd);
	b42 = (mRkd / mLlkd)*(mDLmd / mLlkd - 1);
	b43 = mRkd*mDLmd / mLlkd;
	c11 = mDLmq*mRkq1 / (mLlkq1*mLlkq1)*(mDLmq / mLlkq1 - 1) + mDLmq*mRkq2 / (mLlkq2*mLlkq2*mLlkq1);
	c12 = mDLmq*mRkq2 / (mLlkq2*mLlkq2)*(mDLmq / mLlkq2 - 1) + mDLmq*mDLmq*mRkq1 / (mLlkq1*mLlkq1*mLlkq2);
	c23 = mDLmd*mRfd / (mLlfd*mLlfd)*(mDLmd / mLlfd - 1) + mDLmd*mDLmd*mRkd / (mLlkd*mLlkd*mLlfd);
	c24 = mDLmd*mRkd / (mLlkd*mLlkd)*(mDLmd / mLlkd - 1) + mDLmd*mDLmd*mRfd / (mLlfd*mLlfd*mLlkd);
	c15 = (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq;
	c25 = (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd;
	c26 = mDLmd*mDLmd / mLlfd;

}


void VoltageBehindReactanceEMT::init(Real om, Real dt,
	Real initActivePower, Real initReactivePower, Real initTerminalVolt, Real initVoltAngle) {

	mResistanceMat <<
		mRs, 0, 0,
		0, mRs, 0,
		0, 0, mRs;

	// steady state per unit initial value
	initStatesInPerUnit(initActivePower, initReactivePower, initTerminalVolt, initVoltAngle);

	mVa = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(0);
	mVb = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(1);
	mVc = inverseParkTransform(mThetaMech, mVq, mVd, mV0)(2);

	mIa = inverseParkTransform(mThetaMech, mIq, mId, mI0)(0);
	mIb = inverseParkTransform(mThetaMech, mIq, mId, mI0)(1);
	mIc = inverseParkTransform(mThetaMech, mIq, mId, mI0)(2);
}

void VoltageBehindReactanceEMT::initStatesInPerUnit(Real initActivePower, Real initReactivePower,
	Real initTerminalVolt, Real initVoltAngle) {

	double init_P = initActivePower / mNomPower;
	double init_Q = initReactivePower / mNomPower;
	double init_S = sqrt(pow(init_P, 2.) + pow(init_Q, 2.));
	double init_vt = initTerminalVolt / mBase_v;
	double init_it = init_S / init_vt;

	// power factor
	double init_pf = acos(init_P / init_S);

	// load angle
	double init_delta = atan(((mLmq + mLl) * init_it * cos(init_pf) - mRs * init_it * sin(init_pf)) /
		(init_vt + mRs * init_it * cos(init_pf) + (mLmq + mLl) * init_it * sin(init_pf)));
	double init_delta_deg = init_delta / DPS_PI * 180;

	// dq stator voltages and currents
	double init_vd = init_vt * sin(init_delta);
	double init_vq = init_vt * cos(init_delta);
	double init_id = init_it * sin(init_delta + init_pf);
	double init_iq = init_it * cos(init_delta + init_pf);

	// rotor voltage and current
	double init_ifd = (init_vq + mRs * init_iq + (mLmd + mLl) * init_id) / mLmd;
	double init_vfd = mRfd * init_ifd;

	// flux linkages
	double init_psid = init_vq + mRs * init_iq;
	double init_psiq = -init_vd - mRs * init_id;
	double init_psifd = (mLmd + mLlfd) * init_ifd - mLmd * init_id;
	double init_psid1 = mLmd * (init_ifd - init_id);
	double init_psiq1 = -mLmq * init_iq;
	double init_psiq2 = -mLmq * init_iq;

	// rotor mechanical variables
	double init_Te = init_P + mRs * pow(init_it, 2.);
	mOmMech = 1;
	mOmMech_hist1 = mOmMech;


	mIq = init_iq;
	mId = init_id;
	mI0 = 0;
	mIfd = init_ifd;
	mIkd = 0;
	mIkq1 = 0;
	mIkq2 = 0;

	mVd = init_vd;
	mVq = init_vq;
	mV0 = 0;
	mVfd = init_vfd;
	mVkd = 0;
	mVkq1 = 0;
	mVkq2 = 0;

	mPsiq = init_psiq;
	mPsid = init_psid;
	mPsi0 = 0;
	mPsifd = init_psifd;
	mPsikd = init_psid1;
	mPsikq1 = init_psiq1;
	mPsikq2 = init_psiq2;

	mDPsiq = mDLmq*(mPsikq1 / mLlkq1) + mDLmq*(mPsikq2 / mLlkq2);
	mDPsid = mDLmd*(mPsifd / mLlfd) + mDLmd*(mPsikd / mLlkd);

	mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) +
		mDLmq*mRkq2*(mDPsiq - mPsikq2) / (mLlkq2*mLlkq2) + (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq*mIq;
	mDVd = -mOmMech*mDPsiq + mDLmd*mRkd*(mDPsid - mPsikd) / (mLlkd*mLlkd) + (mDLmd / mLlfd)*mVfd +
		mDLmd*mRfd*(mDPsid - mPsifd) / (mLlfd*mLlfd) + (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd*mId;

	// Initialize mechanical angle
	mThetaMech = initVoltAngle + init_delta;
	//mThetaMech = initVoltAngle + init_delta - M_PI / 2;
	mThetaMech_hist1 = mThetaMech;

	mDVa = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(0);
	mDVb = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(1);
	mDVc = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(2);

	//Initial inductance matrix
	mDInductanceMat <<
	mLl + mLa - mLb*cos(2 * mThetaMech), -mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3),
	-mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), mLl + mLa - mLb*cos(2 * mThetaMech - 4 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech),
	-mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech), mLl + mLa - mLb*cos(2 * mThetaMech + 4 * PI / 3);

	mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

}


void VoltageBehindReactanceEMT::step(SystemModel& system, Real fieldVoltage, Real mechPower, Real time) {

	stepInPerUnit(system.getOmega(), system.getTimeStep(), fieldVoltage, mechPower, time, system.getNumMethod());



	//if (time < 0.1 || time > 0.2)
	//{ 
		//mIa = -(mVa / 1037.8378)*mBase_Z;
		//mIb = -(mVb / 1037.8378)*mBase_Z;
		//mIc = -(mVc / 1037.8378)*mBase_Z;
	//}

	//else {
	//	mIa = (mVa / 10.3)*mBase_Z;
	//	mIb = (mVb / 10.3)*mBase_Z;
	//	mIc = (mVc / 10.3)*mBase_Z;
	//}

}

void VoltageBehindReactanceEMT::stepInPerUnit(Real om, Real dt, Real fieldVoltage, Real mechPower, Real time, NumericalMethod numMethod) {
	


	mVabc <<
		mVa,
		mVb,
		mVc;

	mIabc <<
		mIa,
		mIb,
		mIc;

	mDVabc <<
		mDVa,
		mDVb,
		mDVc;


	//mV_hist = (mResistanceMat - (2. / dt)*mDInductanceMat)*mIabc + mDVabc - mVabc;

	/*mThetaMech_hist2 = mThetaMech_hist1;
	mThetaMech_hist1 = mThetaMech;
	mThetaMech = 2 * mThetaMech - mThetaMech_hist2;

	mOmMech_hist2 = mOmMech_hist1;
	mOmMech_hist1 = mOmMech;
	mOmMech = 2 * mOmMech - mOmMech_hist2;*/


	//Form the Thevinin Equivalent circuit
	//FormTheveninEquivalent(dt);

	//Solve circuit
	//mVabc=R_vbr_eq

	mMechPower = mechPower / mNomPower;
	mMechTorque = mMechPower / mOmMech;

	mElecTorque = (mPsimd*mIq - mPsimq*mId);
	//mElecTorque = (mDPsid*mIq - mDPsiq*mId);


	// Euler step forward	
	mOmMech = mOmMech + dt * (1. / (2 * mH) * (mMechTorque - mElecTorque));

	mThetaMech = mThetaMech + dt * (mOmMech* mBase_OmMech);
	

	mDInductanceMat <<
		mLl + mLa - mLb*cos(2 * mThetaMech), -mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3),
		-mLa / 2 - mLb*cos(2 * mThetaMech - 2 * PI / 3), mLl + mLa - mLb*cos(2 * mThetaMech - 4 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech),
		-mLa / 2 - mLb*cos(2 * mThetaMech + 2 * PI / 3), -mLa / 2 - mLb*cos(2 * mThetaMech), mLl + mLa - mLb*cos(2 * mThetaMech + 4 * PI / 3);

	DPSMatrix R_load(3, 3);

	if (time < 0.1 || time > 0.2)
	{ 
		R_load <<
			1037.8378 / mBase_Z, 0, 0,
			0, 1037.8378 / mBase_Z, 0,
			0, 0, 1037.8378 / mBase_Z;
	}
	else
	{
		R_load <<
			0.001 / mBase_Z, 0, 0,
			0, 0.001 / mBase_Z, 0,
			0, 0, 0.001 / mBase_Z;
	}

	
	mIabc = mIabc - dt*mBase_OmElec*mDInductanceMat.inverse()*(mDVabc+(mResistanceMat + R_load)*mIabc);
	mVabc = -R_load*mIabc;

	mIa = mIabc(0);
	mIb = mIabc(1);
	mIc = mIabc(2);

	mVa = mVabc(0);
	mVb = mVabc(1);
	mVc = mVabc(2);

	mIq = parkTransform(mThetaMech, mIa, mIb, mIc)(0);
	mId = parkTransform(mThetaMech, mIa, mIb, mIc)(1);
	mI0 = parkTransform(mThetaMech, mIa, mIb, mIc)(2);	

	mPsimq = mDLmq*(mPsikq1 / mLlkq1 + mPsikq2 / mLlkq2 + mIq);
	mPsimd = mDLmd*(mPsifd / mLlfd + mPsikd / mLlkd + mId);

	mPsikq1 = mPsikq1 - dt*mBase_OmElec*(mRkq1 / mLlkq1)*(mPsikq1 - mPsimq);
	mPsikq2 = mPsikq2 - dt*mBase_OmElec*(mRkq2 / mLlkq2)*(mPsikq2 - mPsimq);
	mPsifd = mPsifd - dt*mBase_OmElec*((mRfd / mLlfd)*(mPsifd - mPsimd) - mVfd);
	mPsikd = mPsikd - dt*mBase_OmElec*(mRkd / mLlkd)*(mPsikd - mPsimd);
	
	// Calculate dynamic flux likages
	mDPsiq = mDLmq*(mPsikq1 / mLlkq1) + mDLmq*(mPsikq2 / mLlkq2);
	mDPsid = mDLmd*(mPsifd / mLlfd) + mDLmd*(mPsikd / mLlkd);
		
	mDVq = mOmMech*mDPsid + mDLmq*mRkq1*(mDPsiq - mPsikq1) / (mLlkq1*mLlkq1) +
		mDLmq*mRkq2*(mDPsiq - mPsikq2) / (mLlkq2*mLlkq2) + (mRkq1 / (mLlkq1*mLlkq1) + mRkq2 / (mLlkq2*mLlkq2))*mDLmq*mDLmq*mIq;
	mDVd = -mOmMech*mDPsiq + mDLmd*mRkd*(mDPsid - mPsikd) / (mLlkd*mLlkd) + (mDLmd / mLlfd)*mVfd +
		mDLmd*mRfd*(mDPsid - mPsifd) / (mLlfd*mLlfd) + (mRfd / (mLlfd*mLlfd) + mRkd / (mLlkd*mLlkd))*mDLmd*mDLmd*mId;

	mDVa = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(0);
	mDVb = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(1);
	mDVc = inverseParkTransform(mThetaMech, mDVq, mDVd, 0)(2);

	//mVabc = (mResistanceMat + (2 / dt)*mDInductanceMat)*mIabc + mDVabc + mV_hist;
	

	//mMechPower = mechPower / mNomPower;
	//mMechTorque = mMechPower / mOmMech;

	//mElecTorque = (mPsimd*mIq - mPsimq*mId);
	//mOmMech_hist = mOmMech;
	//// Euler step forward	
	//mOmMech = mOmMech + (dt / 2) * (1 / (2 * mH) * (2 * mMechTorque - mElecTorque - mElecTorque_hist));

}



void VoltageBehindReactanceEMT::FormTheveninEquivalent(Real dt) {

	mPsikq1kq2 <<
		mPsikq1,
		mPsikq2;

	mPsifdkd <<
		mPsifd,
		mPsikd;

	c21_omega = -mOmMech*mDLmq / mLlkq1;
	c22_omega = -mOmMech*mDLmq / mLlkq2;
	c13_omega = mOmMech*mDLmd / mLlfd;
	c14_omega = mOmMech*mDLmd / mLlkd;

	Ea <<
		2 - dt*b11, -dt*b12,
		-dt*b21, 2 - dt*b22;
	E1b <<
		dt*b13,
		dt*b23;
	E1 = Ea.inverse() * E1b;


	E2b <<
		2 + dt*b11, dt*b12,
		dt*b21, 2 + dt*b22;;
	E2 = Ea.inverse() * E2b;

	Fa <<
		2 - dt*b31, -dt*b32,
		-dt*b41, 2 - dt*b42;
	F1b <<
		dt*b33,
		dt*b43;
	F1 = Fa.inverse() * F1b;


	E2b <<
		2 + dt*b31, dt*b32,
		dt*b41, 2 + dt*b42;;
	F2 = Fa.inverse() * F2b;

	F3b <<
		2 * dt,
		0;
	F3 = Fa.inverse()*F3b;

	K1a <<
		c11, c12,
		c21_omega, c22_omega;
	K1b <<
		c15,
		0;
	
	K1 = K1a*E1 + K1b;

	K2a <<
		c13_omega, c14_omega,
		c23, c24;
	K1b <<
		0,
		c25;

	K2 = K2a*F1 + K2b;

	ParkMat <<
		2. / 3. * cos(mThetaMech), 2. / 3. * cos(mThetaMech - 2. * M_PI / 3.), 2. / 3. * cos(mThetaMech + 2. * M_PI / 3.),
		2. / 3. * sin(mThetaMech), 2. / 3. * sin(mThetaMech - 2. * M_PI / 3.), 2. / 3. * sin(mThetaMech + 2. * M_PI / 3.),
		1. / 3., 1. / 3., 1. / 3.;

	InverseParkMat <<
		cos(mThetaMech), sin(mThetaMech), 1,
		cos(mThetaMech - 2. * M_PI / 3.), sin(mThetaMech - 2. * M_PI / 3.), 1,
		cos(mThetaMech + 2. * M_PI / 3.), sin(mThetaMech + 2. * M_PI / 3.), 1;

	K <<
		K1, K2, 0,
		0, 0, 0,
		0, 0, 0;
	K = InverseParkMat*K*ParkMat;

	R_vbr_eq = mResistanceMat + (2 / dt)*mDInductanceMat + K;

	C26 <<
		0,
		c26;

	H_qdr = K1a*E2*mPsikq1kq2 + K1a*E1*mIq + K2a*F2*mPsifdkd + K2a*F1*mId + (K2a*F3 + C26)*mVfd;

	
	e_r_vbr <<
		H_qdr,
		0;
	e_r_vbr = InverseParkMat*e_r_vbr;

	e_h_vbr = e_r_vbr + mV_hist;
}

void VoltageBehindReactanceEMT::postStep(SystemModel& system) {
	

}


DPSMatrix VoltageBehindReactanceEMT::parkTransform(Real theta, double a, double b, double c) {

	DPSMatrix dq0vector(3, 1);

	double q, d, zero;

	q = 2. / 3. * cos(theta) * a + 2. / 3. * cos(theta - 2. * M_PI / 3.)*b + 2. / 3. * cos(theta + 2. * M_PI / 3.)*c;
	d = 2. / 3. * sin(theta)*a + 2. / 3. * sin(theta - 2. * M_PI / 3.)*b + 2. / 3. * sin(theta + 2. * M_PI / 3.)*c;
	zero = 1. / 3. * a, 1. / 3. * b, 1. / 3. * c;

	dq0vector << q,
		d,
		0;

	return dq0vector;
}





DPSMatrix VoltageBehindReactanceEMT::inverseParkTransform(Real theta, double q, double d, double zero) {

	DPSMatrix abcVector(3, 1);

	double a, b, c;

	a = cos(theta)*q + sin(theta)*d + 1.*zero;
	b = cos(theta - 2. * M_PI / 3.)*q + sin(theta - 2. * M_PI / 3.)*d + 1.*zero;
	c = cos(theta + 2. * M_PI / 3.)*q + sin(theta + 2. * M_PI / 3.)*d + 1.*zero;

	abcVector << a,
		b,
		c;

	return abcVector;
}

//DPSMatrix VoltageBehindReactanceEMT::parkTransform(Real theta, DPSMatrix& in) {
//	DPSMatrix ParkMat(3, 3);
//	// Park transform according to Krause
//	ParkMat <<
//		2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * M_PI / 3.), 2. / 3. * cos(theta + 2. * M_PI / 3.),
//		2. / 3. * sin(theta), 2. / 3. * sin(theta - 2. * M_PI / 3.), 2. / 3. * sin(theta + 2. * M_PI / 3.),
//		1. / 3., 1. / 3., 1. / 3.;
//
//	// Park transform according to Kundur
//	// ParkMat << 2. / 3. * cos(theta), 2. / 3. * cos(theta - 2. * M_PI / 3.), 2. / 3. * cos(theta + 2. * M_PI / 3.),
//	//	- 2. / 3. * sin(theta), - 2. / 3. * sin(theta - 2. * M_PI / 3.), - 2. / 3. * sin(theta + 2. * M_PI / 3.),
//	//	1. / 3., 1. / 3., 1. / 3.;
//
//	return ParkMat * in;
//}
//
//DPSMatrix VoltageBehindReactanceEMT::inverseParkTransform(Real theta, DPSMatrix& in) {
//	DPSMatrix InverseParkMat(3, 3);
//	// Park transform according to Krause
//	InverseParkMat <<
//		cos(theta), sin(theta), 1,
//		cos(theta - 2. * M_PI / 3.), sin(theta - 2. * M_PI / 3.), 1,
//		cos(theta + 2. * M_PI / 3.), sin(theta + 2. * M_PI / 3.), 1;
//
//	return InverseParkMat * in;
//}