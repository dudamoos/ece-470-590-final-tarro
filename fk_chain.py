
LB = 0.38
LS = 0.48/2
LW = 0.17/2

LA1 = 0.31
LA2 = 0.31
LA3 = 0.13

LL1 = 0.33
LL2 = 0.33
LL3 = 0.12

# root is at center of waist, though waist joint is unused
K_LSP = ([0,  LS ,  LB ], [0, 1, 0])
K_RSP = ([0, -LS ,  LB ], [0, 1, 0])
K_SR  = ([0,  0  ,  0  ], [1, 0, 0])
K_SY  = ([0,  0  ,  0  ], [0, 0, 1])
K_EB  = ([0,  0  , -LA1], [0, 1, 0])
K_WY  = ([0,  0  ,  0  ], [0, 0, 1])
K_WP  = ([0,  0  , -LA2], [0, 1, 0])
K_WR  = ([0,  0  ,  0  ], [1, 0, 0])
K_EH  = ([0,  0  , -LA3], [0, 0, 0])

K_LHY = ([0,  LW ,  0  ], [0, 0, 1])
K_RHY = ([0, -LW ,  0  ], [0, 0, 1])
K_HR  = ([0,  0  ,  0  ], [1, 0, 0])
K_HP  = ([0,  0  ,  0  ], [0, 1, 0])
K_KN  = ([0,  0  , -LL1], [0, 1, 0])
K_AP  = ([0,  0  , -LL2], [0, 1, 0])
K_AR  = ([0,  0  ,  0  ], [1, 0, 0])
K_EF  = ([0,  0  , -LL3], [0, 0, 0])

KLH = [K_LSP, K_SR, K_SY, K_EB, K_WY, K_WP, K_WR, K_EH]
KRH = [K_RSP, K_SR, K_SY, K_EB, K_WY, K_WP, K_WR, K_EH]
KLF = [K_LHY, K_HR, K_HP, K_KN,       K_AP, K_AR, K_EF]
KRF = [K_RHY, K_HR, K_HP, K_KN,       K_AP, K_AR, K_EF]

