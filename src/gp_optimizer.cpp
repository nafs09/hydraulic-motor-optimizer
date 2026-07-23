#include "gp_optimizer.h"
#include "design_vector.h"
#include "config.h"
#include <algorithm>
#include <numeric>
#include <cmath>

static double sq(double x) { return x*x; }

// ── feature extraction ────────────────────────────────────────────────────────
std::vector<double> GPOptimizer::to_features(const Design& d) {
    auto x = design_vector::pack(d);
    for (int i = 0; i < config::N_CTRL; ++i)  x[i] /= config::STROKE;
    x[config::N_CTRL + 0] /= M_PI;
    x[config::N_CTRL + 1] /= 1.0;
    x[config::N_CTRL + 2] /= 0.2;
    return x;
}

// ── kernel ────────────────────────────────────────────────────────────────────
double GPOptimizer::kernel(const std::vector<double>& a,
                            const std::vector<double>& b) const {
    double ssd = 0.0;
    size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) ssd += sq(a[i]-b[i]);
    return m_signal_var * std::exp(-ssd / (2.0*sq(m_length_scale)));
}

// ── triangular solvers ────────────────────────────────────────────────────────
void GPOptimizer::fwd_sub(const std::vector<double>& Lf, int n,
                           const std::vector<double>& b, std::vector<double>& v) {
    const int cap = GP_MAX_POINTS;
    v.assign(n, 0.0);
    for (int i=0;i<n;++i) {
        double s=b[i];
        for (int j=0;j<i;++j) s-=Lf[i*cap+j]*v[j];
        v[i]=s/Lf[i*cap+i];
    }
}
void GPOptimizer::bck_sub(const std::vector<double>& Lf, int n,
                           const std::vector<double>& b, std::vector<double>& v) {
    const int cap = GP_MAX_POINTS;
    v.assign(n, 0.0);
    for (int i=n-1;i>=0;--i) {
        double s=b[i];
        for (int j=i+1;j<n;++j) s-=Lf[j*cap+i]*v[j];
        v[i]=s/Lf[i*cap+i];
    }
}

// ── rebuild alpha ─────────────────────────────────────────────────────────────
void GPOptimizer::rebuild_alpha() {
    m_y_mean = std::accumulate(m_y.begin(),m_y.end(),0.0)/(double)m_n;
    double var=0.0;
    for (double v:m_y) var+=sq(v-m_y_mean);
    m_y_std=(m_n>1&&var>1e-12)?std::sqrt(var/(double)m_n):1.0;
    std::vector<double> yn(m_n);
    for (int i=0;i<m_n;++i) yn[i]=(m_y[i]-m_y_mean)/m_y_std;
    std::vector<double> tmp;
    fwd_sub(m_L_mat,m_n,yn,tmp);
    bck_sub(m_L_mat,m_n,tmp,m_alpha);
}

// ── constructor ───────────────────────────────────────────────────────────────
GPOptimizer::GPOptimizer(double ls, double sv, double nv)
    : m_length_scale(ls), m_signal_var(sv), m_noise_var(nv),
      m_cap(GP_MAX_POINTS), m_n(0),
      m_best_cost(std::numeric_limits<double>::infinity())
{
    m_L_mat.assign(m_cap*m_cap, 0.0);
}

// ── update — records history point ───────────────────────────────────────────
void GPOptimizer::update(const Design& d, double cost) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (cost < m_best_cost) { m_best_cost=cost; m_best_design=d; }

    // Drop worst point if at capacity
    if (m_n == m_cap) {
        int worst=(int)(std::max_element(m_y.begin(),m_y.end())-m_y.begin());
        m_X.erase(m_X.begin()+worst);
        m_y.erase(m_y.begin()+worst);
        m_n--;
        m_L_mat.assign(m_cap*m_cap,0.0);
        for (int i=0;i<m_n;++i) {
            for (int j=0;j<=i;++j) {
                double s=kernel(m_X[i],m_X[j]);
                if (i==j) s+=m_noise_var;
                for (int k=0;k<j;++k) s-=L(i,k)*L(j,k);
                L(i,j)=(i==j)?std::sqrt(std::max(s,1e-12)):s/L(j,j);
            }
        }
    }

    auto feat=to_features(d);
    m_X.push_back(feat);
    m_y.push_back(cost);

    const int row=m_n;
    std::vector<double> k_star(row);
    for (int j=0;j<row;++j) k_star[j]=kernel(feat,m_X[j]);
    std::vector<double> v;
    if (row>0) fwd_sub(m_L_mat,row,k_star,v);
    double diag=kernel(feat,feat)+m_noise_var;
    for (int j=0;j<row;++j) diag-=v[j]*v[j];
    diag=std::sqrt(std::max(diag,1e-12));
    for (int j=0;j<row;++j) L(row,j)=v[j];
    L(row,row)=diag;
    m_n++;
    rebuild_alpha();

    // ── Record convergence history ────────────────────────────────────────
    // Mean uncertainty = average posterior std-dev over last 20 training pts
    // (proxy for how much the GP still disagrees with itself)
    double mean_sigma = 0.0;
    {
        int n_sample = std::min(m_n, 20);
        int start    = m_n - n_sample;
        for (int i = start; i < m_n; ++i) {
            // Variance = k(x,x) - ||L^-1 k_*||^2
            std::vector<double> ki(i);
            for (int j=0;j<i;++j) ki[j]=kernel(m_X[i],m_X[j]);
            std::vector<double> vi;
            if (i>0) fwd_sub(m_L_mat,i,ki,vi);
            double var=kernel(m_X[i],m_X[i]);
            for (auto vj:vi) var-=vj*vj;
            var=std::max(var,1e-12)*sq(m_y_std);
            mean_sigma+=std::sqrt(var);
        }
        mean_sigma=(n_sample>0)?mean_sigma/(double)n_sample:0.0;
    }

    ConvergencePoint cp;
    cp.observation      = (int)m_history.size()+1;
    cp.cost             = cost;
    cp.best_so_far      = m_best_cost;
    cp.mean_uncertainty = mean_sigma;
    m_history.push_back(cp);
}

// ── predict ───────────────────────────────────────────────────────────────────
std::pair<double,double> GPOptimizer::predict(const Design& d) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_n==0) return {0.0,std::sqrt(m_signal_var)};
    auto feat=to_features(d);
    std::vector<double> k_star(m_n);
    for (int i=0;i<m_n;++i) k_star[i]=kernel(feat,m_X[i]);
    double mu_n=0.0;
    for (int i=0;i<m_n;++i) mu_n+=k_star[i]*m_alpha[i];
    double mu=mu_n*m_y_std+m_y_mean;
    std::vector<double> v;
    fwd_sub(m_L_mat,m_n,k_star,v);
    double var=kernel(feat,feat);
    for (auto vj:v) var-=vj*vj;
    var=std::max(var,1e-12)*sq(m_y_std);
    return {mu,std::sqrt(var)};
}

double GPOptimizer::acquisition(const Design& d,double kappa) const {
    auto [mu,sig]=predict(d); return mu-kappa*sig;
}

int GPOptimizer::num_observations() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_n;
}
double GPOptimizer::best_cost() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_best_cost;
}
const Design& GPOptimizer::best_design() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_best_design;
}
std::vector<ConvergencePoint> GPOptimizer::convergence_history() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_history;
}
