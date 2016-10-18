mode(-1)
clf
clear



f = fscanfMat("./states.txt")
f1 = f(:,1)
f2 = f(:,2)
f3 = f(:,3)
state = f(:,4)

//k = [1:length(f1)]
//k = [1800:2000]
k = [1000:1200]

subplot(411)
plot2d(k, f1(k))

subplot(412)
plot2d(k,f2(k))

subplot(413)
plot2d(k,f3(k))

subplot(414)
plot2d(k,state(k))
