function [Minv,C,Jt] = f_evalMinvCJt(qpos,qvel)
Minv(1,1) = -(28147497671065600*(38142987942232112101853170059613818909760263455936*cos(2*qpos(3)) + 23816969724862440834575467389798680003266904925504*cos(2*qpos(4)) - (8342184659454400772715415094551822322787100721817*cos(2*qpos(3) + 2*qpos(4)))/2 - 184814036884226245469752957621394731617737425266953/2))/(28443539075257166360526732495886115124051638608840695938163184128*cos(2*qpos(2) + 2*qpos(3)) - 165861650600450597384184577574530638848782465656077644082580782272*cos(2*qpos(3)) - 141974019334763486003684735291765848686789405190764482820483684048*cos(2*qpos(4)) - 3110417774474913665018787364234798157723357991026411937935587308*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)) - 248316630773440221477495568392011037616311494789271022512543264637*cos(2*qpos(2)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) - 2*qpos(4)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) + 2*qpos(4)) + (36275304895526604283290727891631572670473431878456285857798666809*cos(2*qpos(3) + 2*qpos(4)))/2 + 1139848127922476864766201305592362207884234560063481071269762979147/2);
Minv(1,2) = (28147497671065600*(38142987942232112101853170059613818909760263455936*cos(qpos(2) + 2*qpos(3)) + 14528553143127863956352482344296058567778889676960*cos(qpos(2) - 2*qpos(4)) + 14528553143127863956352482344296058567778889676960*cos(qpos(2) + 2*qpos(4)) + 38142987942232112101853170059613818909760263455936*cos(2*qpos(3)) + 23816969724862440834575467389798680003266904925504*cos(2*qpos(4)) - (8342184659454400772715415094551822322787100721817*cos(2*qpos(3) + 2*qpos(4)))/2 - (215142859091144299812763584882012514751035612277321*cos(qpos(2)))/2 - (8342184659454400772715415094551822322787100721817*cos(qpos(2) + 2*qpos(3) + 2*qpos(4)))/2 - 184814036884226245469752957621394731617737425266953/2))/(28443539075257166360526732495886115124051638608840695938163184128*cos(2*qpos(2) + 2*qpos(3)) - 165861650600450597384184577574530638848782465656077644082580782272*cos(2*qpos(3)) - 141974019334763486003684735291765848686789405190764482820483684048*cos(2*qpos(4)) - 3110417774474913665018787364234798157723357991026411937935587308*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)) - 248316630773440221477495568392011037616311494789271022512543264637*cos(2*qpos(2)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) - 2*qpos(4)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) + 2*qpos(4)) + (36275304895526604283290727891631572670473431878456285857798666809*cos(2*qpos(3) + 2*qpos(4)))/2 + 1139848127922476864766201305592362207884234560063481071269762979147/2);
Minv(1,3) = -(28147497671065600*(38142987942232112101853170059613818909760263455936*cos(qpos(2) + 2*qpos(3)) - 89205339557293127816308367069336517051139957555616*cos(qpos(2) - qpos(3)) + 14528553143127863956352482344296058567778889676960*cos(qpos(2) - 2*qpos(4)) + 14528553143127863956352482344296058567778889676960*cos(qpos(2) + 2*qpos(4)) - 9288416581734576878222985045502621435488015248544*cos(qpos(2) + qpos(3) + 2*qpos(4)) + 70788247850169796715722854611546829170332630121888*cos(qpos(2) + qpos(3)) - (215142859091144299812763584882012514751035612277321*cos(qpos(2)))/2 + 14528553143127863956352482344296058567778889676960*cos(qpos(3) - qpos(2) + 2*qpos(4)) - (8342184659454400772715415094551822322787100721817*cos(qpos(2) + 2*qpos(3) + 2*qpos(4)))/2))/(28443539075257166360526732495886115124051638608840695938163184128*cos(2*qpos(2) + 2*qpos(3)) - 165861650600450597384184577574530638848782465656077644082580782272*cos(2*qpos(3)) - 141974019334763486003684735291765848686789405190764482820483684048*cos(2*qpos(4)) - 3110417774474913665018787364234798157723357991026411937935587308*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)) - 248316630773440221477495568392011037616311494789271022512543264637*cos(2*qpos(2)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) - 2*qpos(4)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) + 2*qpos(4)) + (36275304895526604283290727891631572670473431878456285857798666809*cos(2*qpos(3) + 2*qpos(4)))/2 + 1139848127922476864766201305592362207884234560063481071269762979147/2);
Minv(1,4) = -(360287970189639680*((13938334305827051221298182354583830789240618368065*cos(qpos(2) - qpos(3)))/2 + (1493454527799112271722728851165423698562200087965*cos(qpos(2) + qpos(3) + qpos(4)))/2 - 2706003742636992969498032265925997553652509959208*cos(qpos(2) + qpos(3) - qpos(4)) + 4232617996237917049676828526525845644948725159720*cos(qpos(2) - qpos(3) + qpos(4)) - (3707093795618515675530365835851035900984937073141*cos(qpos(3) - qpos(2) + qpos(4)))/2 + (1451315090896027637222341413359784599295002382585*cos(qpos(2) + qpos(3) + 2*qpos(4)))/2 - (11060663726589030736831696033054192057864473456545*cos(qpos(2) + qpos(3)))/2 - (2270086428613728743180075366296259151215451512025*cos(qpos(3) - qpos(2) + 2*qpos(4)))/2))/(28443539075257166360526732495886115124051638608840695938163184128*cos(2*qpos(2) + 2*qpos(3)) - 165861650600450597384184577574530638848782465656077644082580782272*cos(2*qpos(3)) - 141974019334763486003684735291765848686789405190764482820483684048*cos(2*qpos(4)) - 3110417774474913665018787364234798157723357991026411937935587308*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)) - 248316630773440221477495568392011037616311494789271022512543264637*cos(2*qpos(2)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) - 2*qpos(4)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) + 2*qpos(4)) + (36275304895526604283290727891631572670473431878456285857798666809*cos(2*qpos(3) + 2*qpos(4)))/2 + 1139848127922476864766201305592362207884234560063481071269762979147/2);
Minv(2,1) = Minv(1,2);
Minv(2,2) = -(28147497671065600*(76285975884464224203706340119227637819520526911872*cos(qpos(2) + 2*qpos(3)) + 29057106286255727912704964688592117135557779353920*cos(qpos(2) - 2*qpos(4)) + 29057106286255727912704964688592117135557779353920*cos(qpos(2) + 2*qpos(4)) + 38142987942232112101853170059613818909760263455936*cos(2*qpos(3)) + 83430670854849016754614455518855363579352187991808*cos(2*qpos(4)) - (8342184659454400772715415094551822322787100721817*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)))/2 + 38142987942232112101853170059613818909760263455936*cos(2*qpos(2) + 2*qpos(3)) - (8342184659454400772715415094551822322787100721817*cos(2*qpos(3) + 2*qpos(4)))/2 - 215142859091144299812763584882012514751035612277321*cos(qpos(2)) - 8342184659454400772715415094551822322787100721817*cos(qpos(2) + 2*qpos(3) + 2*qpos(4)) - 288406060646880146505399808049740396797550404243653))/(28443539075257166360526732495886115124051638608840695938163184128*cos(2*qpos(2) + 2*qpos(3)) - 165861650600450597384184577574530638848782465656077644082580782272*cos(2*qpos(3)) - 141974019334763486003684735291765848686789405190764482820483684048*cos(2*qpos(4)) - 3110417774474913665018787364234798157723357991026411937935587308*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)) - 248316630773440221477495568392011037616311494789271022512543264637*cos(2*qpos(2)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) - 2*qpos(4)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) + 2*qpos(4)) + (36275304895526604283290727891631572670473431878456285857798666809*cos(2*qpos(3) + 2*qpos(4)))/2 + 1139848127922476864766201305592362207884234560063481071269762979147/2);
Minv(2,3) = (28147497671065600*(38142987942232112101853170059613818909760263455936*cos(qpos(2) + 2*qpos(3)) - 89205339557293127816308367069336517051139957555616*cos(qpos(2) - qpos(3)) + 89205339557293127816308367069336517051139957555616*cos(2*qpos(2) + qpos(3)) + 14528553143127863956352482344296058567778889676960*cos(qpos(2) - 2*qpos(4)) + 14528553143127863956352482344296058567778889676960*cos(qpos(2) + 2*qpos(4)) + 45085147986858711963686505784760625008306393389344*cos(qpos(3) + 2*qpos(4)) + 59613701129986575920038988129056683576085283066304*cos(2*qpos(4)) - (8342184659454400772715415094551822322787100721817*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)))/2 + 38142987942232112101853170059613818909760263455936*cos(2*qpos(2) + 2*qpos(3)) - 9288416581734576878222985045502621435488015248544*cos(qpos(2) + qpos(3) + 2*qpos(4)) + 70788247850169796715722854611546829170332630121888*cos(qpos(2) + qpos(3)) - (215142859091144299812763584882012514751035612277321*cos(qpos(2)))/2 - 196600176104219130490847512559815679446451493568288*cos(qpos(3)) + 14528553143127863956352482344296058567778889676960*cos(qpos(3) - qpos(2) + 2*qpos(4)) - (8342184659454400772715415094551822322787100721817*cos(qpos(2) + 2*qpos(3) + 2*qpos(4)))/2 - 14528553143127863956352482344296058567778889676960*cos(2*qpos(2) + qpos(3) + 2*qpos(4)) - 391998084409534047541046658478086061977363383220353/2))/(28443539075257166360526732495886115124051638608840695938163184128*cos(2*qpos(2) + 2*qpos(3)) - 165861650600450597384184577574530638848782465656077644082580782272*cos(2*qpos(3)) - 141974019334763486003684735291765848686789405190764482820483684048*cos(2*qpos(4)) - 3110417774474913665018787364234798157723357991026411937935587308*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)) - 248316630773440221477495568392011037616311494789271022512543264637*cos(2*qpos(2)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) - 2*qpos(4)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) + 2*qpos(4)) + (36275304895526604283290727891631572670473431878456285857798666809*cos(2*qpos(3) + 2*qpos(4)))/2 + 1139848127922476864766201305592362207884234560063481071269762979147/2);
Minv(2,4) = (180143985094819840*(13938334305827051221298182354583830789240618368065*cos(qpos(2) - qpos(3)) - 13938334305827051221298182354583830789240618368065*cos(2*qpos(2) + qpos(3)) + 26269402995919292451614310900277174319788603049616*cos(qpos(3) - qpos(4)) - 7044554372946673744326016528868847657547873967085*cos(qpos(3) + 2*qpos(4)) + 1493454527799112271722728851165423698562200087965*cos(qpos(2) + qpos(3) + qpos(4)) - 5412007485273985938996064531851995107305019918416*cos(qpos(2) + qpos(3) - qpos(4)) + 8465235992475834099353657053051691289897450319440*cos(qpos(2) - qpos(3) + qpos(4)) - 3707093795618515675530365835851035900984937073141*cos(qpos(3) - qpos(2) + qpos(4)) + 1451315090896027637222341413359784599295002382585*cos(qpos(2) + qpos(3) + 2*qpos(4)) + 3707093795618515675530365835851035900984937073141*cos(2*qpos(2) + qpos(3) + qpos(4)) - 11060663726589030736831696033054192057864473456545*cos(qpos(2) + qpos(3)) - 16615398272886859598847533276124703712641084407390*cos(qpos(3) + qpos(4)) + 30718777516284239139194923837471199913508045870045*cos(qpos(3)) - 2270086428613728743180075366296259151215451512025*cos(qpos(3) - qpos(2) + 2*qpos(4)) - 8465235992475834099353657053051691289897450319440*cos(2*qpos(2) + qpos(3) - qpos(4)) + 2270086428613728743180075366296259151215451512025*cos(2*qpos(2) + qpos(3) + 2*qpos(4))))/(28443539075257166360526732495886115124051638608840695938163184128*cos(2*qpos(2) + 2*qpos(3)) - 165861650600450597384184577574530638848782465656077644082580782272*cos(2*qpos(3)) - 141974019334763486003684735291765848686789405190764482820483684048*cos(2*qpos(4)) - 3110417774474913665018787364234798157723357991026411937935587308*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)) - 248316630773440221477495568392011037616311494789271022512543264637*cos(2*qpos(2)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) - 2*qpos(4)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) + 2*qpos(4)) + (36275304895526604283290727891631572670473431878456285857798666809*cos(2*qpos(3) + 2*qpos(4)))/2 + 1139848127922476864766201305592362207884234560063481071269762979147/2);
Minv(3,1) = Minv(1,3);
Minv(3,2) = Minv(2,3);
Minv(3,3) = -(28147497671065600*(178410679114586255632616734138673034102279915111232*cos(2*qpos(2) + qpos(3)) + 90170295973717423927373011569521250016612786778688*cos(qpos(3) + 2*qpos(4)) + 203091424322379730380199092492830877153487378148096*cos(2*qpos(2)) + 59613701129986575920038988129056683576085283066304*cos(2*qpos(4)) - (18822457782240974928974409692138696587368849578649*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)))/2 + 38142987942232112101853170059613818909760263455936*cos(2*qpos(2) + 2*qpos(3)) + 30556594843730848007334023440464566440527503712384*cos(2*qpos(3) + 2*qpos(4)) - 393200352208438260981695025119631358892902987136576*cos(qpos(3)) - 29057106286255727912704964688592117135557779353920*cos(2*qpos(2) + qpos(3) + 2*qpos(4)) - 1147424077306737069955303606934650775245233755698561/2))/(28443539075257166360526732495886115124051638608840695938163184128*cos(2*qpos(2) + 2*qpos(3)) - 165861650600450597384184577574530638848782465656077644082580782272*cos(2*qpos(3)) - 141974019334763486003684735291765848686789405190764482820483684048*cos(2*qpos(4)) - 3110417774474913665018787364234798157723357991026411937935587308*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)) - 248316630773440221477495568392011037616311494789271022512543264637*cos(2*qpos(2)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) - 2*qpos(4)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) + 2*qpos(4)) + (36275304895526604283290727891631572670473431878456285857798666809*cos(2*qpos(3) + 2*qpos(4)))/2 + 1139848127922476864766201305592362207884234560063481071269762979147/2);
Minv(3,4) = (180143985094819840*(13938334305827051221298182354583830789240618368065*cos(2*qpos(2) + qpos(3)) - 26269402995919292451614310900277174319788603049616*cos(qpos(3) - qpos(4)) + 16573337702673657506214964285720748023882635618240*cos(2*qpos(2) + qpos(4)) + 7044554372946673744326016528868847657547873967085*cos(qpos(3) + 2*qpos(4)) + 17804167003443458352260653847225483029891152730176*cos(2*qpos(3) + qpos(4)) + 31733035050371832871906108202004824555232402835640*cos(2*qpos(2)) - 818771337717701105957733952936474551920449129440*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)) + 16573337702673657506214964285720748023882635618240*cos(2*qpos(2) - qpos(4)) + 4774467944332945001145941162572588506332422455060*cos(2*qpos(3) + 2*qpos(4)) - 3707093795618515675530365835851035900984937073141*cos(2*qpos(2) + qpos(3) + qpos(4)) + 16615398272886859598847533276124703712641084407390*cos(qpos(3) + qpos(4)) - 30718777516284239139194923837471199913508045870045*cos(qpos(3)) - 70004191389769249119589141730478430153626524224320*cos(qpos(4)) + 8465235992475834099353657053051691289897450319440*cos(2*qpos(2) + qpos(3) - qpos(4)) - 2270086428613728743180075366296259151215451512025*cos(2*qpos(2) + qpos(3) + 2*qpos(4)) - 3053228507201848160357592521199696182592430401024*cos(2*qpos(2) + 2*qpos(3) + qpos(4)) - 59017655695093986126113824098169118224052372849860))/(28443539075257166360526732495886115124051638608840695938163184128*cos(2*qpos(2) + 2*qpos(3)) - 165861650600450597384184577574530638848782465656077644082580782272*cos(2*qpos(3)) - 141974019334763486003684735291765848686789405190764482820483684048*cos(2*qpos(4)) - 3110417774474913665018787364234798157723357991026411937935587308*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)) - 248316630773440221477495568392011037616311494789271022512543264637*cos(2*qpos(2)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) - 2*qpos(4)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) + 2*qpos(4)) + (36275304895526604283290727891631572670473431878456285857798666809*cos(2*qpos(3) + 2*qpos(4)))/2 + 1139848127922476864766201305592362207884234560063481071269762979147/2);
Minv(4,1) = Minv(1,4);
Minv(4,2) = Minv(2,4);
Minv(4,3) = Minv(3,4);
Minv(4,4) = -(288230376151711744*(20716672128342071882768705357150935029853294522800*cos(2*qpos(2) + qpos(4)) + 22255208754304322940325817309031853787363940912720*cos(2*qpos(3) + qpos(4)) + 75375121449714800069850096465038784417889675892331*cos(2*qpos(2)) + 41495247007411863858242067100188512336365074710656*cos(2*qpos(3)) - 511732086073563191223583720585296594950280705900*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)) - 7116001049187441852601000358110495228259151596544*cos(2*qpos(2) + 2*qpos(3)) + 20716672128342071882768705357150935029853294522800*cos(2*qpos(2) - qpos(4)) + (5968084930416181251432426453215735632915528068825*cos(2*qpos(3) + 2*qpos(4)))/2 - 87505239237211561399486427163098037692033155280400*cos(qpos(4)) - 3816535634002310200446990651499620228240538001280*cos(2*qpos(2) + 2*qpos(3) + qpos(4)) - 323584413924958956984720632804441179219330719161813/2))/(28443539075257166360526732495886115124051638608840695938163184128*cos(2*qpos(2) + 2*qpos(3)) - 165861650600450597384184577574530638848782465656077644082580782272*cos(2*qpos(3)) - 141974019334763486003684735291765848686789405190764482820483684048*cos(2*qpos(4)) - 3110417774474913665018787364234798157723357991026411937935587308*cos(2*qpos(2) + 2*qpos(3) + 2*qpos(4)) - 248316630773440221477495568392011037616311494789271022512543264637*cos(2*qpos(2)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) - 2*qpos(4)) + 30038002506082475795838687330968440082003180088731492735785849800*cos(2*qpos(2) + 2*qpos(4)) + (36275304895526604283290727891631572670473431878456285857798666809*cos(2*qpos(3) + 2*qpos(4)))/2 + 1139848127922476864766201305592362207884234560063481071269762979147/2);

C(1,1) = - (7409379798061167*qvel(2)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(3)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(4)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 - (69074626144050807*qvel(2)*sin(qpos(2) + qpos(3)))/450359962737049600 - (69074626144050807*qvel(3)*sin(qpos(2) + qpos(3)))/450359962737049600 - (7409379798061167*qvel(3)*sin(qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(4)*sin(qpos(3) + qpos(4)))/180143985094819840 - (270470216492207943*qvel(2)*sin(qpos(2)))/900719925474099200 - (69074626144050807*qvel(3)*sin(qpos(3)))/450359962737049600 - (7409379798061167*qvel(4)*sin(qpos(4)))/180143985094819840;
C(1,2) = - (7409379798061167*qvel(1)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(2)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(3)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(4)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 - (69074626144050807*qvel(1)*sin(qpos(2) + qpos(3)))/450359962737049600 - (69074626144050807*qvel(2)*sin(qpos(2) + qpos(3)))/450359962737049600 - (69074626144050807*qvel(3)*sin(qpos(2) + qpos(3)))/450359962737049600 - (7409379798061167*qvel(3)*sin(qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(4)*sin(qpos(3) + qpos(4)))/180143985094819840 - (270470216492207943*qvel(1)*sin(qpos(2)))/900719925474099200 - (270470216492207943*qvel(2)*sin(qpos(2)))/900719925474099200 - (69074626144050807*qvel(3)*sin(qpos(3)))/450359962737049600 - (7409379798061167*qvel(4)*sin(qpos(4)))/180143985094819840;
C(1,3) = - (7409379798061167*qvel(1)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(2)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(3)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(4)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 - (69074626144050807*qvel(1)*sin(qpos(2) + qpos(3)))/450359962737049600 - (69074626144050807*qvel(2)*sin(qpos(2) + qpos(3)))/450359962737049600 - (7409379798061167*qvel(1)*sin(qpos(3) + qpos(4)))/180143985094819840 - (69074626144050807*qvel(3)*sin(qpos(2) + qpos(3)))/450359962737049600 - (7409379798061167*qvel(2)*sin(qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(3)*sin(qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(4)*sin(qpos(3) + qpos(4)))/180143985094819840 - (69074626144050807*qvel(1)*sin(qpos(3)))/450359962737049600 - (69074626144050807*qvel(2)*sin(qpos(3)))/450359962737049600 - (69074626144050807*qvel(3)*sin(qpos(3)))/450359962737049600 - (7409379798061167*qvel(4)*sin(qpos(4)))/180143985094819840;
C(1,4) = -(7409379798061167*(sin(qpos(2) + qpos(3) + qpos(4)) + sin(qpos(3) + qpos(4)) + sin(qpos(4)))*(qvel(1) + qvel(2) + qvel(3) + qvel(4)))/180143985094819840;
C(2,1) = (7409379798061167*qvel(1)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 + (69074626144050807*qvel(1)*sin(qpos(2) + qpos(3)))/450359962737049600 - (7409379798061167*qvel(3)*sin(qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(4)*sin(qpos(3) + qpos(4)))/180143985094819840 + (270470216492207943*qvel(1)*sin(qpos(2)))/900719925474099200 - (69074626144050807*qvel(3)*sin(qpos(3)))/450359962737049600 - (7409379798061167*qvel(4)*sin(qpos(4)))/180143985094819840;
C(2,2) = - (7409379798061167*qvel(3)*sin(qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(4)*sin(qpos(3) + qpos(4)))/180143985094819840 - (69074626144050807*qvel(3)*sin(qpos(3)))/450359962737049600 - (7409379798061167*qvel(4)*sin(qpos(4)))/180143985094819840;
C(2,3) = - (7409379798061167*qvel(1)*sin(qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(2)*sin(qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(3)*sin(qpos(3) + qpos(4)))/180143985094819840 - (7409379798061167*qvel(4)*sin(qpos(3) + qpos(4)))/180143985094819840 - (69074626144050807*qvel(1)*sin(qpos(3)))/450359962737049600 - (69074626144050807*qvel(2)*sin(qpos(3)))/450359962737049600 - (69074626144050807*qvel(3)*sin(qpos(3)))/450359962737049600 - (7409379798061167*qvel(4)*sin(qpos(4)))/180143985094819840;
C(2,4) = -(7409379798061167*(sin(qpos(3) + qpos(4)) + sin(qpos(4)))*(qvel(1) + qvel(2) + qvel(3) + qvel(4)))/180143985094819840;
C(3,1) = (7409379798061167*qvel(1)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 + (69074626144050807*qvel(1)*sin(qpos(2) + qpos(3)))/450359962737049600 + (7409379798061167*qvel(1)*sin(qpos(3) + qpos(4)))/180143985094819840 + (7409379798061167*qvel(2)*sin(qpos(3) + qpos(4)))/180143985094819840 + (69074626144050807*qvel(1)*sin(qpos(3)))/450359962737049600 + (69074626144050807*qvel(2)*sin(qpos(3)))/450359962737049600 - (7409379798061167*qvel(4)*sin(qpos(4)))/180143985094819840;
C(3,2) = (7409379798061167*qvel(1)*sin(qpos(3) + qpos(4)))/180143985094819840 + (7409379798061167*qvel(2)*sin(qpos(3) + qpos(4)))/180143985094819840 + (69074626144050807*qvel(1)*sin(qpos(3)))/450359962737049600 + (69074626144050807*qvel(2)*sin(qpos(3)))/450359962737049600 - (7409379798061167*qvel(4)*sin(qpos(4)))/180143985094819840;
C(3,3) = -(7409379798061167*qvel(4)*sin(qpos(4)))/180143985094819840;
C(3,4) = -(7409379798061167*sin(qpos(4))*(qvel(1) + qvel(2) + qvel(3) + qvel(4)))/180143985094819840;
C(4,1) = (7409379798061167*qvel(1)*sin(qpos(2) + qpos(3) + qpos(4)))/180143985094819840 + (7409379798061167*qvel(1)*sin(qpos(3) + qpos(4)))/180143985094819840 + (7409379798061167*qvel(2)*sin(qpos(3) + qpos(4)))/180143985094819840 + (7409379798061167*qvel(1)*sin(qpos(4)))/180143985094819840 + (7409379798061167*qvel(2)*sin(qpos(4)))/180143985094819840 + (7409379798061167*qvel(3)*sin(qpos(4)))/180143985094819840;
C(4,2) = (7409379798061167*qvel(1)*sin(qpos(3) + qpos(4)))/180143985094819840 + (7409379798061167*qvel(2)*sin(qpos(3) + qpos(4)))/180143985094819840 + (7409379798061167*qvel(1)*sin(qpos(4)))/180143985094819840 + (7409379798061167*qvel(2)*sin(qpos(4)))/180143985094819840 + (7409379798061167*qvel(3)*sin(qpos(4)))/180143985094819840;
C(4,3) = (7409379798061167*sin(qpos(4))*(qvel(1) + qvel(2) + qvel(3)))/180143985094819840;
C(4,4) = 0;

Jt(1,1) = - (3*cos(qpos(1) + qpos(2) + qpos(3)))/10 - cos(qpos(1) + qpos(2) + qpos(3) + qpos(4))/8 - (3*cos(qpos(1) + qpos(2)))/10 - (3*cos(qpos(1)))/10;
Jt(2,1) = - (3*sin(qpos(1) + qpos(2) + qpos(3)))/10 - sin(qpos(1) + qpos(2) + qpos(3) + qpos(4))/8 - (3*sin(qpos(1) + qpos(2)))/10 - (3*sin(qpos(1)))/10;
Jt(1,2) = - (3*cos(qpos(1) + qpos(2) + qpos(3)))/10 - cos(qpos(1) + qpos(2) + qpos(3) + qpos(4))/8 - (3*cos(qpos(1) + qpos(2)))/10;
Jt(2,2) = - (3*sin(qpos(1) + qpos(2) + qpos(3)))/10 - sin(qpos(1) + qpos(2) + qpos(3) + qpos(4))/8 - (3*sin(qpos(1) + qpos(2)))/10;
Jt(1,3) = - (3*cos(qpos(1) + qpos(2) + qpos(3)))/10 - cos(qpos(1) + qpos(2) + qpos(3) + qpos(4))/8;
Jt(2,3) = - (3*sin(qpos(1) + qpos(2) + qpos(3)))/10 - sin(qpos(1) + qpos(2) + qpos(3) + qpos(4))/8;
Jt(1,4) = -cos(qpos(1) + qpos(2) + qpos(3) + qpos(4))/8;
Jt(2,4) = -sin(qpos(1) + qpos(2) + qpos(3) + qpos(4))/8;

