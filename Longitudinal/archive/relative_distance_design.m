

caseno = 1;

switch caseno
  case 1
    sys = tf(1,[1 2 1])*tf(1,[1 0]);
    kp = 0.25;
    ki_by_kp = 0.25;
    leadlag = tf([1/0.5 1],[1/2 1]);
  case 2
    w1 = 8;
    w2 = 2;
    sys = tf(w1*w2,[1 w1 w1*w2])*tf(1,[1 0]);
    kp = 0.25;
    ki_by_kp = 0.25;
    leadlag = tf([1/0.25 1],[1/1 1]);        
  end


claw = kp*tf([1 ki_by_kp],[1 0])*leadlag;
L = claw*sys;
clp = L/(1+L);

figure(1)
bode(L);

figure(2);
bode(clp)

figure(3)
bode(leadlag)

figure(4)
step(clp)

