#include <CPL-SLAM/CPL-SLAM.h>
#include <SESync/SESync.h>

#include <fstream>

void exportCertificateResultsSingleCSV(
        const SESync::SESyncResult & R,
        const std::string& filename)
{
  std::ofstream out(filename);

  // 1) Export SDPval vector entries
  for (size_t k = 0; k < R.SDPvalVector.size(); ++k) {
    out << "SDPval," << k << ",," << R.SDPvalVector[k] << "\n";
  }

  // 2) Export gradnorm vector entries
  for (size_t k = 0; k < R.gradnormVector.size(); ++k) {
    out << "gradnorm," << k << ",," << R.gradnormVector[k] << "\n";
  }

  // 3) Export scalar fields
  out << "startingRank : "         << R.rank_iters.front()         << "\n";
  out << "endingRank : "           << R.rank_iters.back()           << "\n";

  // 4) Export initialization_time vector entries
  out << "initialization_time : " << R.initialization_time << "\n";

  // 5) Export elapsed_optimization_times vector entries
  double FinalOptTime = 0;
  for (size_t k = 0; k < R.elapsed_optimization_times.size(); ++k) {
    out << "elapsed_optimization_times," << k << ","
        << R.elapsed_optimization_times[k].back() << "\n";
    FinalOptTime += R.elapsed_optimization_times[k].back();
  }

  // 6) Export verification_times vector entries
  for (size_t k = 0; k < R.verification_times.size(); ++k) {
    out << "verification_times," << k << ",,"
        << R.verification_times[k] << "\n";
  }

  out << "Final optimization time : " << FinalOptTime << "\n";
  out << "Final total computation time : " << R.total_computation_time << "\n";

  std::cout << "All fields exported to \"" << filename << "\"\n";
}

int main(int argc, char *argv[]) {
  bool use_CPL = true;
  bool write = false;

  if (argc < 2 || argc > 4) {
    std::cout << "Usage: " << argv[0] << " [input .g2o file]" << std::endl;
    exit(1);
  }

  if (argc >= 3) {
    std::string alg(argv[2]);

    if (alg == "CPL") {
      use_CPL = true;
    } else if (alg == "SE") {
      use_CPL = false;
    } else {
      std::cout << "The second argument must be either \"CPL\" or \"SE\""
                << std::endl;
      exit(1);
    }
  }
    std::string outputPath;
    if (argc >= 3) {
//    write = true;
      outputPath = argv[3];
  }

  size_t num_poses;

  if (use_CPL) {
    // PGO with CPL-SLAM
    CPL_SLAM::measurements_t measurements = CPL_SLAM::read_g2o_file(argv[1]);

    CPL_SLAM::CPL_SLAMOpts opts;



    opts.initialization = CPL_SLAM::Initialization::Random;
    opts.r0 = 2;
    opts.verbose = true;  // Print output to stdout
    opts.reg_Cholesky_precon_max_condition_number = 2e6;

#if defined(_OPENMP)
    opts.num_threads = 4;
#endif

    CPL_SLAM::CPL_SLAMResult results = CPL_SLAM::CPL_SLAM(measurements, opts);

    if (write) {
      std::string filename(argv[3]);
      std::cout << "Saving final poses to file: " << filename << std::endl;
      std::ofstream poses_file(filename);

      CPL_SLAM::RealMatrix X(
          2, measurements.num_landmarks + 3 * measurements.num_poses);

      const size_t size = measurements.num_landmarks + measurements.num_poses;

      const CPL_SLAM::RealVector c = results.xhat.real();
      const CPL_SLAM::RealVector s = results.xhat.imag();

      X.topLeftCorner(1, size) = c.head(size).transpose();
      X.bottomLeftCorner(1, size) = s.head(size).transpose();

      for (size_t n = 0; n < measurements.num_poses; n++) {
        size_t j = size + 2 * n;
        size_t k = size + n;
        X(0, j) = c[k];
        X(1, j) = s[k];
        X(0, j + 1) = -s[k];
        X(1, j + 1) = c[k];
      }

      poses_file << X << std::endl << std::endl << std::endl;

      Eigen::Matrix<size_t, 2, Eigen::Dynamic> edges;

      edges.setZero(2, measurements.first.size());

      for (size_t e = 0; e < measurements.first.size(); e++) {
        edges(0, e) = measurements.first[e].i;
        edges(1, e) = measurements.first[e].j;
      }

      poses_file << edges << std::endl << std::endl << std::endl;

      edges.setZero(2, measurements.second.size());

      for (size_t e = 0; e < measurements.second.size(); e++) {
        edges(0, e) = measurements.second[e].i;
        edges(1, e) = measurements.second[e].j;
      }

      poses_file << edges << std::endl << std::endl << std::endl;

      poses_file.close();
    }
  } else {
    // PGO with SE-Sync
    SESync::measurements_t measurements = SESync::read_g2o_file(argv[1]);

    SESync::SESyncOpts opts;
    opts.r0 = 2;
    opts.verbose = true;  // Print output to stdout
    opts.reg_Cholesky_precon_max_condition_number = 2e6;
    opts.initialization = SESync::Initialization::Random;
//    opts.formulation = SESync::Formulation::Explicit;
#if defined(_OPENMP)
    opts.num_threads = 4;
#endif

    SESync::SESyncResult results = SESync::SESync(measurements, opts);
    exportCertificateResultsSingleCSV(results, outputPath);

    if (write) {
      std::string filename(argv[3]);
      std::cout << "Saving final poses to file: " << filename << std::endl;
      std::ofstream poses_file(filename);
      poses_file << results.xhat;
      poses_file.close();
    }
  }

  return 0;
}
