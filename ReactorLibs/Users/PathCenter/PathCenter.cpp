#include "PathCenter.hpp"

#include "ttd_b1.hpp"
#include "ttd_b2.hpp"
#include "ttd_b3.hpp"
#include "ttd_b4.hpp"
#include "ttd_b5.hpp"
#include "ttd_b6.hpp"

#include "sttrb1.hpp"
#include "sttrb2.hpp"
#include "sttrb3.hpp"
#include "sttrb4.hpp"
#include "sttrb5.hpp"
#include "sttrb6.hpp"

#include "sttr1.hpp"
#include "sttr2.hpp"
#include "sttr3.hpp"
#include "sttr4.hpp"
#include "sttr5.hpp"
#include "sttr6.hpp"

#include "ttd_1.hpp"
#include "ttd_2.hpp"
#include "ttd_3.hpp"
#include "ttd_4.hpp"

#include "dtt_1.hpp"
#include "dtt_2.hpp"
#include "dtt_3.hpp"

#include "dtt_b_4.hpp"
#include "dtt_b_5.hpp"
#include "dtt_b_6.hpp"

#include "a12_1.hpp"
#include "a12_2.hpp"
#include "a12_3.hpp"

#include "a12_b1.hpp"
#include "a12_b2.hpp"
#include "a12_b3.hpp"

#include "a23_2.hpp"
#include "a23_b2.hpp"

#include "FFM_B.hpp"
#include "FFM.hpp"

#include "w_to_n4.hpp"
#include "w_to_n5.hpp"
#include "w_to_n6.hpp"
#include "w_to_comb.hpp"

#include "n4_back.hpp"
#include "n5_back.hpp"
#include "n6_back.hpp"

#include "wb_to_n4.hpp"
#include "wb_to_n5.hpp"
#include "wb_to_n6.hpp"

#include "n4b_back.hpp"
#include "n5b_back.hpp"
#include "n6b_back.hpp"

#include "a23_retry.hpp"
#include "a23_bretry.hpp"

namespace PATH
{
const Path<20>& TTD_B1()
{
    return ::TTD_B1;
}

const Path<20>& TTD_B2()
{
    return ::TTD_B2;
}

const Path<21>& TTD_B3()
{
    return ::TTD_B3;
}

const Path<21>& TTD_B4()
{
    return ::TTD_B4;
}

const Path<21>& TTD_B5()
{
    return ::TTD_B5;
}

const Path<23>& TTD_B6()
{
    return ::TTD_B6;
}

const Path<44>& sttr_b1()
{
    return ::sttr_b1;
}

const Path<47>& sttr_b2()
{
    return ::sttr_b2;
}

const Path<54>& sttr_b3()
{
    return ::sttr_b3;
}

const Path<60>& sttr_b4()
{
    return ::sttr_b4;
}

const Path<68>& sttr_b5()
{
    return ::sttr_b5;
}

const Path<75>& sttr_b6()
{
    return ::sttr_b6;
}

const Path<44>& sttr1()
{
    return ::sttr1;
}

const Path<47>& sttr2()
{
    return ::sttr2;
}

const Path<55>& sttr3()
{
    return ::sttr3;
}

const Path<66>& sttr4()
{
    return ::sttr4;
}

const Path<74>& sttr5()
{
    return ::sttr5;
}

const Path<82>& sttr6()
{
    return ::sttr6;
}

const Path<26>& TTD_1()
{
    return ::TTD_1;
}

const Path<23>& TTD_2()
{
    return ::TTD_2;
}

const Path<21>& TTD_3()
{
    return ::TTD_3;
}

const Path<21>& TTD_4()
{
    return ::TTD_4;
}

const Path<72>& DTT_1()
{
    return ::DTT_1;
}

const Path<71>& DTT_2()
{
    return ::DTT_2;
}

const Path<67>& DTT_3()
{
    return ::DTT_3;
}

const Path<68>& DTT_B4()
{
    return ::DTT_B4;
}

const Path<55>& DTT_B5()
{
    return ::DTT_B5;
}

const Path<57>& DTT_B6()
{
    return ::DTT_B6;
}

const Path<68>& A12_1()
{
    return ::A12_1;
}

const Path<108>& A12_2()
{
    return ::A12_2;
}

const Path<147>& A12_3()
{
    return ::A12_3;
}

const Path<67>& A12_B1()
{
    return ::A12_B1;
}

const Path<108>& A12_B2()
{
    return ::A12_B2;
}

const Path<147>& A12_B3()
{
    return ::A12_B3;
}

const Path<188>& A23_2()
{
    return ::A23_2;
}

const Path<188>& A23_B2()
{
    return ::A23_B2;
}
const Path<38>& FFM_B()
{
    return ::FFM_B;
}
const Path<38>& FFM()
{
    return ::FFM;
}

const Path<181>& W_TO_N4()
{
    return ::W_to_N4;
}

const Path<161>& W_TO_N5()
{
    return ::W_to_N5;
}

const Path<156>& W_TO_N6()
{
    return ::W_to_N6;
}

const Path<180>& N4_BACK()
{
    return ::N4_Back;
}

const Path<161>& N5_BACK()
{
    return ::N5_Back;
}

const Path<156>& N6_BACK()
{
    return ::N6_Back;
}

const Path<114>& W_TO_COMB()
{
    return ::W_to_Comb;
}

const Path<181>& WB_TO_N4()
{
    return ::WB_to_N4;
}

const Path<161>& WB_TO_N5()
{
    return ::WB_to_N5;
}

const Path<156>& WB_TO_N6()
{
    return ::WB_to_N6;
}

const Path<180>& N4B_BACK()
{
    return ::N4B_Back;
}

const Path<161>& N5B_BACK()
{
    return ::N5B_Back;
}

const Path<156>& N6B_BACK()
{
    return ::N6B_Back;
}

const Path<202>& A23_Retry()
{
    return ::A23_RETRY;
}

const Path<202>& A23_B_Retry()
{
    return ::A23_B_RETRY;
}

}


