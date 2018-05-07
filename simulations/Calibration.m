c = 299702547;
function ret = fun(v)
  d12 = 25289.63;
  d13 = 25354.62;
  d14 = 25553.89;
  d15 = 25587.38;
  d16 = 25705.51;
  d23 = 25062.55;
  d24 = 24881.05;
  d25 = 24868.09;
  d26 = 24819.05;
  d34 = 25268.67;
  d35 = 25215;
  d36 = 25203.28;
  d45 = 25336.18;
  d46 = 25495.26;
  d56 = 25704.3;
  d = 1855;
  EDMactual = d*(ones(6) - eye(6));
  EDMmeasured = [0, d12, d13, d14, d15, d16;
              d12, 0, d23, d24, d25, d26;
              d13, d23, 0, d34, d35, d36;
              d14, d24, d34, 0, d45, d46;
              d15, d25, d35, d45, 0, d56;
              d16, d26, d36, d46, d56, 0];
  rangeBiais = -101.65;
  EDMdecaRanging = EDMmeasured + rangeBiais*(ones(6) - eye(6));
  
  c = 299702547;
  # TOF in ns
  TOFActual = 1000000 * EDMactual / c;
  TOFMeasured = 1000000 * EDMdecaRanging / c;
  
  A = [0, (v(1)+v(2)), (v(1)+v(3)), (v(1)+v(4)), (v(1)+v(5)), (v(1)+v(6));
       0, 0, (v(2)+v(3)), (v(2)+v(4)), (v(2)+v(5)), (v(2)+v(6));
       0, 0, 0, (v(3)+v(4)), (v(3)+v(5)), (v(3)+v(6));
       0, 0, 0, 0, (v(4)+v(5)), (v(4)+v(6));
       0, 0, 0, 0, 0, (v(5)+v(6));
       0, 0, 0, 0, 0, 0];
  A = (A + A') / 2;

  ret = norm(TOFActual - TOFMeasured + A);
endfunction

[x] = fminsearch(@fun, 77*ones(6,1));
delays = x * 299702547 / 1000000
