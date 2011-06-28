/*
 * =====================================================================================
 *
 *       Filename:  createIndex.cpp
 *
 *    Description:  Various lshkit tools put together for creating lshkit indices
 *
 *        Version:  1.0
 *        Created:  12/28/2010 06:56:58 PM CET
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Javier Romero (jrgn), jrgn@kth.se
 *        Company:  CAS/CSC KTH
 *
 * =====================================================================================
 */

#include <cstdlib>
#include <iostream>
#include <gsl/gsl_multifit.h>
//#include <boost/program_options.hpp>
//#include <boost/progress.hpp>
#include <boost/format.hpp>
#include <boost/timer.hpp>
#include <boost/program_options.hpp>
#include <lshkit.h>
#include <lshkit/tune.h>

namespace po = boost::program_options; 
using namespace std;
using namespace lshkit;
typedef MultiProbeLshIndex<unsigned> Index;
//using namespace tr1;

    bool is_good_value (double v) {
        return ((v > -std::numeric_limits<double>::max())
                && (v < std::numeric_limits<double>::max()));
    }

static const int MIN_L = 1;
static const int MAX_L = 20;

static const int MIN_T = 1;
static const int MAX_T = Probe::MAX_T;

static const int MIN_M = 1;
static const int MAX_M = Probe::MAX_M;

static const double MIN_W = 0.01;
static const double MAX_W = 10;
static const double NUM_W = 400;
static const double DELTA_W = (MAX_W - MIN_W) / NUM_W;

// L, T, M, W
tune::Interval intervals[]= {{MIN_L, MAX_L + 1},
    {MIN_T, MAX_T + 1},
    {0, MAX_M - MIN_M + 1},
    {0, static_cast<unsigned>(NUM_W) + 1}};

static double gTargetRecall;
static MultiProbeLshDataModel *gModel;
static double gRadius;

double recall_K (const tune::Input &x) {
    gModel->setL(x[0]);
    gModel->setT(x[1]);
    gModel->setM(MAX_M - x[2]);
    gModel->setW(MIN_W + DELTA_W * x[3]);
    return gModel->avgRecall();
}

double recall_R (const tune::Input &x) {
    gModel->setL(x[0]);
    gModel->setT(x[1]);
    gModel->setM(MAX_M - x[2]);
    gModel->setW(MIN_W + DELTA_W * x[3]);
    return gModel->recall(gRadius);
}

double cost (const tune::Input &x) {
    gModel->setL(x[0]);
    gModel->setT(x[1]);
    gModel->setM(MAX_M - x[2]);
    gModel->setW(MIN_W + DELTA_W * x[3]);
    return gModel->cost();
}

bool constraint_K (const tune::Input &x) {
    return recall_K(x) > gTargetRecall;
}

bool constraint_R (const tune::Input &x) {
    return recall_R(x) > gTargetRecall;
}

void mytune(const unsigned pNBins,const unsigned pNTables,const unsigned pNPoints,
        const double gM,const double gG,const gsl_vector *vM,const gsl_vector *vG,
        int &pM,double &pW,
        const float pRecall=0.9,const unsigned pK=100,const float pRadius=-1.0)
{
    std::cout << pNBins << " " << pNTables << " " << pNPoints << " " << gM << " " << gG << " " << pRecall << " " << pK << " " << pRadius << std::endl;
    gTargetRecall=pRecall;
    gRadius=pRadius;
    bool do_K = true;

    if (pRadius>0) {
        do_K = false;
    }

    std::cout << "intervals" << std::endl;
    intervals[0].begin = pNTables;
    intervals[0].end = pNTables + 1;
    intervals[1].begin = pNBins;
    intervals[1].end = pNBins + 1;

    std::cout << "scale" << std::endl;
    DataParam param(gM,gG,vM,vG);
    double scale = param.scale();

    std::cout << "local model" << std::endl;
    MultiProbeLshDataModel local_model(param, pNPoints, pK);
    gModel = &local_model;


    std::cout << "intervals" << std::endl;
    int begin_M = intervals[2].begin;
    int end_M = intervals[2].end;

    std::cout << "looping" << std::endl;
    for (int m = begin_M; m < end_M; ++m) {

        intervals[2].begin = m;
        intervals[2].end = m + 1;

        tune::Range range(intervals, intervals + sizeof intervals /sizeof intervals[0]);
        tune::Input input;
        std::cout << "tunning" << std::endl;
        bool ok = do_K ? tune::Tune(range, constraint_K, &input)
            : tune::Tune(range, constraint_R, &input);

        if (ok) {
            pM=MAX_M - input[2];
            pW=(MIN_W + DELTA_W * input[3]) * sqrt(scale);
        } else {
            cerr << "Failed." << endl;
            exit(-1);
        }

    }
}

void fitdata(const std::string pDataPath,double &gM,double &gG,
        gsl_vector *pM,gsl_vector *pG,const unsigned pNPoints=0,
        const unsigned pNPairs=50000,unsigned pNQueries=1000,
        unsigned pK=100,const unsigned pNFolds=10)
{
    /* load matrix */
    Matrix<float> data(pDataPath);

    vector<unsigned> idx(data.getSize());
    for (unsigned i = 0; i < idx.size(); ++i) idx[i] = i;
    random_shuffle(idx.begin(), idx.end());

    if (pNPoints > 0 && pNPoints < data.getSize()) idx.resize(pNPoints);

    metric::l2sqr<float> l2sqr(data.getDim());

    DefaultRng rng;
    boost::variate_generator<DefaultRng &, UniformUnsigned> gen(rng,
            UniformUnsigned(0, idx.size()-1));

    {
        // sample pNPairs pairs of points
        for (unsigned k = 0; k < pNPairs; ++k)
        {
            double dist, logdist;
            for (;;)
            {
                unsigned i = gen();
                unsigned j = gen();
                if (i == j) continue;
                dist = l2sqr(data[idx[i]], data[idx[j]]);
                logdist = log(dist);
                if (is_good_value(logdist)) break;
            }
            gM += dist;
            gG += logdist;
        }
        gM /= pNPairs;
        gG /= pNPairs;
        gG = exp(gG);
    }

    if (pNQueries > idx.size()) pNQueries = idx.size();
    if (pK > idx.size() - pNQueries) pK = idx.size() - pNQueries;
    /* sample query */
    vector<unsigned> qry(pNQueries);

    SampleQueries(&qry, idx.size(), rng);

    /* do the queries */
    vector<Topk<unsigned> > topks(pNQueries);
    for (unsigned i = 0; i < pNQueries; ++i) topks[i].reset(pK);

    /* ... */
    gsl_matrix *X = gsl_matrix_alloc(pNFolds * pK, 3);
    gsl_vector *yM = gsl_vector_alloc(pNFolds * pK);
    gsl_vector *yG = gsl_vector_alloc(pNFolds * pK);
    gsl_matrix *cov = gsl_matrix_alloc(3,3);

    vector<double> M(pK);
    vector<double> G(pK);

    std::cout << "looping" << std::endl;
    unsigned m = 0;
    for (unsigned l = 0; l < pNFolds; l++)
    {
        // Scan
        for (unsigned i = l; i< idx.size(); i += pNFolds)
        {
            for (unsigned j = 0; j < pNQueries; j++) 
            {
                int id = qry[j];
                if (i != id) 
                {
                    float d = l2sqr(data[idx[id]], data[idx[i]]);
                    if (is_good_value(log(double(d)))) topks[j] << Topk<unsigned>::Element(i, d);
                }
            }
        }

        fill(M.begin(), M.end(), 0.0);
        fill(G.begin(), G.end(), 0.0);

        for (unsigned i = 0; i < pNQueries; i++)
        {
            for (unsigned k = 0; k < pK; k++)
            {
                M[k] += topks[i][k].dist;
                G[k] += log(topks[i][k].dist);
            }
        }

        for (unsigned k = 0; k < pK; k++)
        {
            M[k] = log(M[k]/pNQueries);
            G[k] /= pNQueries;
            gsl_matrix_set(X, m, 0, 1.0);
            gsl_matrix_set(X, m, 1, log(double(data.getSize() * (l + 1)) / double(pNFolds)));
            gsl_matrix_set(X, m, 2, log(double(k + 1)));
            gsl_vector_set(yM, m, M[k]);
            gsl_vector_set(yG, m, G[k]);
            ++m;
        }
    }

    gsl_multifit_linear_workspace *work = gsl_multifit_linear_alloc(pNFolds * pK, 3);

    double chisq;

    std::cout << "fitting1" << std::endl;
    gsl_multifit_linear(X, yM, pM, cov, &chisq, work);
    std::cout << "fitting2" << std::endl;
    gsl_multifit_linear(X, yG, pG, cov, &chisq, work);

    // to be returned: gM gG pM pG
    //    cout << gM << '\t' << gG << endl;
    //    cout << gsl_vector_get(pM, 0) << '\t'
    //         << gsl_vector_get(pM, 1) << '\t'
    //         << gsl_vector_get(pM, 2) << endl;
    //    cout << gsl_vector_get(pG, 0) << '\t'
    //         << gsl_vector_get(pG, 1) << '\t'
    //         << gsl_vector_get(pG, 2) << endl;

    std::cout << "freeing" << std::endl;
    gsl_matrix_free(X);
    gsl_matrix_free(cov);
    gsl_vector_free(yM);
    gsl_vector_free(yG);
}

const Index run(const std::string pDataPath,const std::string pIndexPath,const float pW,
        const unsigned pM,const unsigned pNTables,const unsigned pHashSize=1017881)
{
    boost::timer timer;

    cout << "LOADING DATA..." << endl;
    timer.restart();
    FloatMatrix data(pDataPath);
    cout << boost::format("LOAD TIME: %1%s.") % timer.elapsed() << endl;


    FloatMatrix::Accessor accessor(data);
    Index index;

    // We define a short name for the MPLSH index.
    Index::Parameter param;

    // Setup the parameters.  Note that pNTables is not provided here.
    param.W = pW;
    param.range = pHashSize; // See H in the program parameters.  You can just use the default value.
    param.repeat = pM;
    param.dim = data.getDim();
    DefaultRng rng;

    index.init(param, rng, pNTables);
    // The accessor.

    // Initialize the index structure.  Note pNTables is passed here.
    cout << "CONSTRUCTING INDEX..." << endl;

    timer.restart();
    {
        for (unsigned i = 0; i < data.getSize(); ++i)
        {
            // Insert an item to the hash table.
            // Note that only the key is passed in here.
            // MPLSH will get the feature from the accessor.
            index.insert(i, data[i]);
        }
    }
    cout << boost::format("CONSTRUCTION TIME: %1%s.") % timer.elapsed() << endl;

    timer.restart();
    cout << "SAVING INDEX..." << endl;
    {
        ofstream os(pIndexPath.c_str(), ios_base::binary);
        os.exceptions(ios_base::eofbit | ios_base::failbit | ios_base::badbit);
        index.save(os);
    }
    cout << boost::format("SAVING TIME: %1%s") % timer.elapsed() << endl;
    return index;
}

int main(int argc,char* argv[])
{
    std::string lDataPath,lIndexPath;
    unsigned lNPoints,lNPairs,lNQueries,lK,lNFolds,lNBins,lNTables;
    double lRecall;

    boost::timer timer;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message.")
        (",T", po::value<unsigned>(&lNBins)->default_value(50), "# bins")
        (",L", po::value<unsigned>(&lNTables)->default_value(10), "# hash tables")
        (",Q", po::value<unsigned>(&lNQueries)->default_value(1000), "# queries to sample")
        (",K", po::value<unsigned>(&lK)->default_value(100), "# nearest neighbor to retrieve")
        ("points,N", po::value<unsigned>(&lNPoints), "# number of data points")
        ("pairs,P", po::value<unsigned>(&lNPairs)->default_value(50000), "# pairs of points to sample")
        (",F", po::value<unsigned>(&lNFolds)->default_value(10), "# divide the sample to F folds")
        ("recall,R", po::value<double>(&lRecall)->default_value(0.9), "desired recall")
        ("data,D", po::value<string>(&lDataPath), "data file")
        ("index,I", po::value<string>(&lIndexPath), "index file")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm); 

    if (vm.count("help") || (vm.count("data") < 1))
    {
        cout << desc;
        return 0;
    }

    gsl_vector *vM = gsl_vector_alloc(3);
    gsl_vector *vG = gsl_vector_alloc(3);
    double gM = 0.0;
    double gG = 0.0;

    std::cout << "fitting data" << std::endl;
    fitdata(lDataPath,gM,gG,vM,vG,lNPoints,lNPairs,lNQueries,lK,lNFolds);
    int lM;
    double lW;
    std::cout << "tunning data: " << gM << " " << gG << " " << 
        gsl_vector_get(vM,0) << " " << gsl_vector_get(vM,1) << " " << 
        gsl_vector_get(vM,2) << " " << gsl_vector_get(vG,0) << " " << 
        gsl_vector_get(vG,1) << " " << gsl_vector_get(vG,2) << std::endl;
    mytune(lNBins,lNTables,lNPoints,gM,gG,vM,vG,lM,lW,lRecall,lK);
    std::cout << "running data: " << lM << " " << lW << std::endl;
    run(lDataPath,lIndexPath,lW,lM,lNTables);
    std::cout << "Index written to " << lIndexPath << std::endl;
    gsl_vector_free(vM);
    gsl_vector_free(vG);
    return 0;
}
