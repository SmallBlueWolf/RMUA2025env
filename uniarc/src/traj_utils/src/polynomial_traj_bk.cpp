// #include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <traj_utils/polynomial_traj.h>

PolynomialTraj PolynomialTraj::minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time)
{
  std::cout << "in minsnap" << std::endl;
  int seg_num = Time.size();
  // Eigen::MatrixXd poly_coeff(seg_num, 3 * 6);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, 50, 18> poly_coeff;//TODO:
  // Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);
  Eigen::MatrixXd Px(6 * seg_num, 1), Py(6 * seg_num, 1), Pz(6 * seg_num, 1);

  int num_f, num_p; // number of fixed and free variables
  int num_d;        // number of all segments' derivatives

  const static auto Factorial = [](int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };

  /* ---------- end point derivative ---------- */
  std::cout << "end point derivative" << std::endl;
  // Eigen::MatrixXd Dx = Eigen::MatrixXd::Zero(seg_num * 6, 1);
  // Eigen::MatrixXd Dy = Eigen::MatrixXd::Zero(seg_num * 6, 1);
  // Eigen::MatrixXd Dz = Eigen::MatrixXd::Zero(seg_num * 6, 1);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, 50, 1> Dx, Dy, Dz;//TODO:
  Dx = Eigen::MatrixXd::Zero(seg_num * 6, 1);
  Dy = Eigen::MatrixXd::Zero(seg_num * 6, 1);
  Dz = Eigen::MatrixXd::Zero(seg_num * 6, 1);


  for (int k = 0; k < seg_num; k++)
  {
    /* position to derivative */
    Dx(k * 6, 0) = Pos(0, k);
    Dx(k * 6 + 1, 0) = Pos(0, k + 1);
    Dy(k * 6, 0) = Pos(1, k);
    Dy(k * 6 + 1, 0) = Pos(1, k + 1);
    Dz(k * 6, 0) = Pos(2, k);
    Dz(k * 6 + 1, 0) = Pos(2, k + 1);

    if (k == 0)
    {
      Dx(k * 6 + 2, 0) = start_vel(0);
      Dy(k * 6 + 2, 0) = start_vel(1);
      Dz(k * 6 + 2, 0) = start_vel(2);

      Dx(k * 6 + 4, 0) = start_acc(0);
      Dy(k * 6 + 4, 0) = start_acc(1);
      Dz(k * 6 + 4, 0) = start_acc(2);
    }
    else if (k == seg_num - 1)
    {
      Dx(k * 6 + 3, 0) = end_vel(0);
      Dy(k * 6 + 3, 0) = end_vel(1);
      Dz(k * 6 + 3, 0) = end_vel(2);

      Dx(k * 6 + 5, 0) = end_acc(0);
      Dy(k * 6 + 5, 0) = end_acc(1);
      Dz(k * 6 + 5, 0) = end_acc(2);
    }
  }

  /* ---------- Mapping Matrix A ---------- */
  std::cout << "Mapping Matrix A" << std::endl;
  // Eigen::MatrixXd Ab;
  // Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, 50, 50> A, Ab;//TODO:
//   A.setZero();
  A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    Ab = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; i++)
    {
      Ab(2 * i, i) = Factorial(i);
      for (int j = i; j < 6; j++)
        Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
    }
    A.block(k * 6, k * 6, 6, 6) = Ab;
  }

  /* ---------- Produce Selection Matrix C' ---------- */
  std::cout << "Produce Selection Matrix C' " << std::endl;
  // Eigen::MatrixXd Ct, C;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, 50, 50> Ct,C;//TODO:

  num_f = 2 * seg_num + 4; // 3 + 3 + (seg_num - 1) * 2 = 2m + 4
  num_p = 2 * seg_num - 2; //(seg_num - 1) * 2 = 2m - 2
  num_d = 6 * seg_num;
  Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
//   Ct.setZero();

  Ct(0, 0) = 1;
  Ct(2, 1) = 1;
  Ct(4, 2) = 1; // stack the start point
  Ct(1, 3) = 1;
  Ct(3, 2 * seg_num + 4) = 1;
  Ct(5, 2 * seg_num + 5) = 1;

  Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
  Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1; // Stack the end point

  for (int j = 2; j < seg_num; j++)
  {
    Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
    Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
    Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
    Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
    Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
  }

  C = Ct.transpose();

  Eigen::MatrixXd Dx1 = C * Dx;
  Eigen::MatrixXd Dy1 = C * Dy;
  Eigen::MatrixXd Dz1 = C * Dz;

  /* ---------- minimum snap matrix ---------- */
  std::cout << "minimun snap matrix" << std::endl;
  // Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, 50, 50> Q;//TODO:
//   Q.setZero();
  Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    for (int i = 3; i < 6; i++)
    {
      for (int j = 3; j < 6; j++)
      {
        Q(k * 6 + i, k * 6 + j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
      }
    }
  }
  std::cout << "Q: " << std::endl << Q << std::endl;

  /* ---------- R matrix ---------- */
  std::cout << "in R matrix" << std::endl;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, 50, 50> R;//TODO:
  // Eigen::MatrixXd R = Eigen::MatrixXd::Zero(C.rows(),C.rows());
  R = Eigen::MatrixXd::Zero(C.rows(),C.rows());
  std::cout << "set R matrix" << std::endl;
  
//   Eigen::MatrixXd ATinv = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);
//   ATinv= A.transpose().inverse();
//   std::cout << "A.transpose.inverse: " << std::endl << ATinv << std::endl;

//   Eigen::MatrixXd Ainv = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);
//   Ainv = A.inverse();
//   std::cout << "A.inverse: " << std::endl << Ainv << std::endl;
  
//   Eigen::MatrixXd Temp = Q * Ainv;
//   std::cout << "temp result1: " << std::endl << Temp << std::endl;
//   Temp = ATinv * Temp;
//   std::cout << "temp result2: " << std::endl << Temp << std::endl;
  R = C * A.transpose().inverse() * Q * A.inverse() * Ct;
//   R = C * ATinv * Q * Ainv * Ct;
  std::cout << "solved the R" << std::endl;

  // Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);
  // Eigen::MatrixXd Dxf(2 * seg_num + 4,1), Dyf(2 * seg_num + 4,1), Dzf(2 * seg_num + 4,1);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, 50, 1> Dxf, Dyf, Dzf;//TODO:

  Dxf = Dx1.block(0, 0, 2 * seg_num + 4, 1);
  Dyf = Dy1.block(0, 0, 2 * seg_num + 4, 1);
  Dzf = Dz1.block(0, 0, 2 * seg_num + 4, 1);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, 50, 50> Rff, Rfp, Rpf, Rpp;//TODO:
  // Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
  // Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
  // Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
  // Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

  Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
  Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
  Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
  Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

  /* ---------- close form solution ---------- */
  std::cout << "in minsnap" << std::endl;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, 50, 1> Dxp, Dyp, Dzp;//TODO:
  // Eigen::MatrixXd Dxp(2 * seg_num - 2, 1), Dyp(2 * seg_num - 2, 1), Dzp(2 * seg_num - 2, 1);
  // Eigen::MatrixXd Dxp(2 * seg_num - 2, 1), Dyp(2 * seg_num - 2, 1), Dzp(2 * seg_num - 2, 1);
  std::cout << "Rpp * Rfp: "<<Rpp.inverse() * Rfp.transpose() << std::endl;
  std::cout << "Rpp * Rfp * Dxf: "<<(Rpp.inverse() * Rfp.transpose()) * Dxf << std::endl;
  Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
  Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
  Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

  // Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
  // Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
  // Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;
  Dx1.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 1) = Dxp;
  Dy1.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 1) = Dyp;
  Dz1.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 1) = Dzp;

  std::cout << "Px" << std::endl;
  Px = (A.inverse() * Ct) * Dx1;
  Py = (A.inverse() * Ct) * Dy1;
  Pz = (A.inverse() * Ct) * Dz1;

  std::cout << "into the for loop: poly_coeff" << std::endl;
  for (int i = 0; i < seg_num; i++)
  {
    // poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
    // poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
    // poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 0, 1, 6) = Px.block(i * 6, 0, 6, 1).transpose();
    poly_coeff.block(i, 6, 1, 6) = Py.block(i * 6, 0, 6, 1).transpose();
    poly_coeff.block(i, 12, 1, 6) = Pz.block(i * 6, 0, 6, 1).transpose();
  }

  /* ---------- use polynomials ---------- */
  std::cout << "use polynomials" << std::endl;
  PolynomialTraj poly_traj;
  poly_traj.init();
  for (int i = 0; i < poly_coeff.rows(); ++i)
  {
    std::cout << "into the for loop: poly_traj" << std::endl;
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j)
    {
      cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6), cz[j] = poly_coeff(i, j + 12);
    }
    std::cout << "out the for loop: poly_traj" << std::endl;
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    double ts = Time(i);
    std::cout << "start addSegment" << std::endl;
    poly_traj.addSegment(cx, cy, cz, ts);

    for (const auto& element : cx) {
        std::cout << element << " ";
    }
    std::cout << std::endl;
    for (const auto& element : cy) {
        std::cout << element << " ";
    }
    std::cout << std::endl;
    for (const auto& element : cz) {
        std::cout << element << " ";
    }
    std::cout << std::endl;
    std::cout << "t = " << ts << std::endl;
  }
  std::cout << "end of the minSnapTraj function---" << std::endl;
  std::cout <<  poly_traj.getTimeSum() << std::endl;
  return poly_traj;
}

PolynomialTraj PolynomialTraj::one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                                    double t)
{
//   Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6), Crow(1, 6);
  Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 1, 6> Crow;
//   Eigen::VectorXd Bx(6), By(6), Bz(6);
  Eigen::Matrix<double, 6, 1> Bx, By, Bz;

  C(0, 5) = 1;
  C(1, 4) = 1;
  C(2, 3) = 2;
  Crow << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;
  C.row(3) = Crow;
  Crow << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0;
  C.row(4) = Crow;
  Crow << 20 * pow(t, 3), 12 * pow(t, 2), 6 * t, 2, 0, 0;
  C.row(5) = Crow;

  Bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0), end_acc(0);
  By << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1), end_acc(1);
  Bz << start_pt(2), start_vel(2), start_acc(2), end_pt(2), end_vel(2), end_acc(2);
  std::cout << "Bx:" << Bx << std::endl;
  std::cout << "By:" << By << std::endl;
  std::cout << "Bz:" << Bz << std::endl;
  std::cout << "C:" << C << std::endl;
//   Eigen::VectorXd Cofx = C.colPivHouseholderQr().solve(Bx);
  Eigen::Matrix<double, 6, 1> Cofx = C.colPivHouseholderQr().solve(Bx);
  Eigen::Matrix<double, 6, 1> Cofy = C.colPivHouseholderQr().solve(By);
  Eigen::Matrix<double, 6, 1> Cofz = C.colPivHouseholderQr().solve(Bz);

  vector<double> cx(6), cy(6), cz(6);
  for (int i = 0; i < 6; i++)
  {
    cx[i] = Cofx(i);
    cy[i] = Cofy(i);
    cz[i] = Cofz(i);
  }

  PolynomialTraj poly_traj;
  poly_traj.addSegment(cx, cy, cz, t);

  return poly_traj;
}