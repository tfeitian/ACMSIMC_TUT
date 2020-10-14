int isNumber(double x)
{
    // This looks like it should always be true,
    // but it's false if x is a NaN (1.#QNAN0).
    return (x == x);
    // see https://www.johndcook.com/blog/IEEE_exceptions_in_cpp/ cb: https://stackoverflow.com/questions/347920/what-do-1-inf00-1-ind00-and-1-ind-mean
}

double LP_Filter(double fin, double fcoef, double *fold)
{
    double ffilter = fin * fcoef + (1 - fcoef) * (*fold);
    *fold = ffilter;
    return ffilter;
}