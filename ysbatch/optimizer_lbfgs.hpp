#include <stdio.h>
#include <lbfgs.h>
#include "../../BaseLib/math/optimize.h"

class LBFGS_METHOD: public Optimize::Method
{
protected:
    lbfgsfloatval_t *m_x;
	lbfgs_parameter_t param;
public:
    LBFGS_METHOD(double epsilon=1e-5) : m_x(NULL)
					 ,Optimize::Method()
    {
		/* Initialize the parameters for the L-BFGS optimization. */
		lbfgs_parameter_init(&param);
		param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;
		param.epsilon=epsilon;
    }

    virtual ~LBFGS_METHOD()
    {
        if (m_x != NULL) {
            lbfgs_free(m_x);
            m_x = NULL;
        }
    }
protected:
	virtual void optimize(vectorn & initial)
    {

		int N=initial.size();
        lbfgsfloatval_t fx;
        lbfgsfloatval_t *m_x = lbfgs_malloc(N);

        if (m_x == NULL) {
            printf("ERROR: Failed to allocate a memory block for variables.\n");
            return ;
        }

        /* Initialize the variables. */
        for (int i = 0;i < N;i ++) {
            m_x[i] = initial[i];
        }


        /*
            Start the L-BFGS optimization; this will invoke the callback functions
            evaluate() and progress() when necessary.
         */
        int ret = lbfgs(N, m_x, &fx, _evaluate, _progress, this, &param);

        /* Report the result. */
        printf("L-BFGS optimization terminated with status code = %d\n", ret);
        //printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, m_x[0], m_x[1]);

        for (int i = 0;i < N;i ++) {
            initial[i]=m_x[i] ;
        }
    }

    static lbfgsfloatval_t _evaluate(
        void *instance,
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
        )
    {
        return reinterpret_cast<LBFGS_METHOD*>(instance)->evaluate(x, g, n, step);
    }

    lbfgsfloatval_t evaluate(
        const lbfgsfloatval_t *x,
        lbfgsfloatval_t *g,
        const int n,
        const lbfgsfloatval_t step
        )
    {
        lbfgsfloatval_t fx = 0.0;

		vectornView mx((double*)x, n, 1);
		vectornView mg((double*)g, n, 1);

		return func_dfunc(mx, mg);
    }

    static int _progress(
        void *instance,
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
        )
    {
        return reinterpret_cast<LBFGS_METHOD*>(instance)->progress(x, g, fx, xnorm, gnorm, step, n, k, ls);
    }

    int progress(
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
        )
    {
        printf("Iteration %d:\n", k);
        printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
        printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
        printf("\n");
        return 0;
    }
};




