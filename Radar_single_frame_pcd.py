#!/usr/bin/env python
import rospy
import math
import sys

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''
    rospy.init_node('pcl2_pub_example')
    pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2)
    rospy.loginfo("Initializing sample pcl2 publisher node...")
    #give time to roscore to make the connections
    rospy.sleep(1.)
    cloud_points = [
[0.241611346602440	,0.0642065629363060	,0.416217833757401],
[0.327343702316284	,0.444893240928650	,0.125613421201706],
[0.291532427072525	,0.428194165229797	,0.229175105690956],
[0.274833351373673	,0.464005440473557	,0.173286482691765],
[0.374107092618942	,0.508449375629425	,0.143558204174042],
[0.333179920911789	,0.489364743232727	,0.261914402246475],
[0.314095258712769	,0.530291914939880	,0.198041692376137],
[0.232240930199623	,0.492122620344162	,0.350664168596268],
[0.213156268000603	,0.533049762248993	,0.299179226160049],
[0.191313758492470	,0.473037958145142	,0.398396790027618],
[0.172229096293449	,0.513965129852295	,0.353924930095673],
[0.153144448995590	,0.554892301559448	,0.296187758445740],
[0.264894515275955	,0.208390265703201	,0.552711725234985],
[0.245809853076935	,0.249317437410355	,0.544521987438202],
[0.243051990866661	,0.148378446698189	,0.581373751163483],
[0.204882681369782	,0.230232790112495	,0.569299399852753],
[0.166713371872902	,0.312087148427963	,0.542120039463043],
[0.420870482921600	,0.572005569934845	,0.161502972245216],
[0.374827414751053	,0.550535321235657	,0.294653713703156],
[0.353357166051865	,0.596578419208527	,0.222796902060509],
[0.390092462301254	,0.277379512786865	,0.548902571201325],
[0.368622213602066	,0.323422580957413	,0.538443148136139],
[0.325681746006012	,0.415508747100830	,0.501686632633209],
[0.304211527109146	,0.461551815271378	,0.474163323640823],
[0.261271029710770	,0.553637921810150	,0.394497185945511],
[0.218330577015877	,0.645724058151245	,0.256471693515778],
[0.408460080623627	,0.117780081927776	,0.591349124908447],
[0.386989861726761	,0.163823157548904	,0.594816267490387],
[0.365519613027573	,0.209866225719452	,0.593933582305908],
[0.344049394130707	,0.255909293889999	,0.588681578636169],
[0.322579145431519	,0.301952362060547	,0.578941285610199],
[0.279638677835465	,0.394038498401642	,0.544923305511475],
[0.258168458938599	,0.440081566572189	,0.519694924354553],
[0.215227976441383	,0.532167732715607	,0.448196411132813],
[0.193757742643356	,0.578210771083832	,0.398165553808212],
[0.172287508845329	,0.624253869056702	,0.333211213350296],
[0.362417012453079	,0.0963098481297493	,0.624326705932617],
[0.319476544857025	,0.188395991921425	,0.626775264739990],
[0.298006325960159	,0.234439060091972	,0.621800720691681],
[0.276536077260971	,0.280482113361359	,0.612587213516235],
[0.233595609664917	,0.372568249702454	,0.580544173717499],
[0.212125375866890	,0.418611317873001	,0.556931734085083],
[0.190655142068863	,0.464654386043549	,0.527401626110077],
[0.316373944282532	,0.0748396143317223	,0.651699423789978],
[0.294903725385666	,0.120882682502270	,0.654847085475922],
[0.273433476686478	,0.166925743222237	,0.654045462608337],
[0.251963257789612	,0.212968826293945	,0.649279892444611],
[0.230493009090424	,0.259011894464493	,0.640461862087250],
[0.209022775292397	,0.305054962635040	,0.627420544624329],
[0.187552541494370	,0.351098030805588	,0.609885036945343],
[0.144612073898315	,0.443184167146683	,0.559536159038544],
[0.227390423417091	,0.145455509424210	,0.676418423652649],
[0.205920174717903	,0.191498577594757	,0.671811521053314],
[0.184449940919876	,0.237541645765305	,0.663293123245239],
[0.141509473323822	,0.329627782106400	,0.633818686008453],
[0.0985690057277679	,0.421713918447495	,0.585531473159790],
[0.484595030546188	,0.332055300474167	,0.556532680988312],
[0.460739225149155	,0.383214265108109	,0.543771922588348],
[0.528859317302704	,0.103563614189625	,0.603658914566040],
[0.505003511905670	,0.154722571372986	,0.613069355487824],
[0.433436065912247	,0.308199465274811	,0.609891772270203],
[0.409580230712891	,0.359358429908752	,0.598270177841187],
[0.361868619918823	,0.461676359176636	,0.557429552078247],
[0.338012784719467	,0.512835323810577	,0.526848137378693],
[0.290301144123077	,0.615153253078461	,0.438330203294754],
[0.501556158065796	,0.0285488292574883	,0.634387910366058],
[0.477700352668762	,0.0797077938914299	,0.648282885551453],
[0.453844547271729	,0.130866765975952	,0.657054543495178],
[0.429988741874695	,0.182025730609894	,0.660906970500946],
[0.382277101278305	,0.284343659877777	,0.654090642929077],
[0.358421266078949	,0.335502624511719	,0.643268108367920],
[0.310709655284882	,0.437820553779602	,0.605470359325409],
[0.286853820085526	,0.488979518413544	,0.577438831329346],
[0.262998014688492	,0.540138483047485	,0.542129278182983],
[0.239142194390297	,0.591297447681427	,0.497996002435684],
[0.402685582637787	,0.107010945677757	,0.693696379661560],
[0.378829777240753	,0.158169910311699	,0.697346389293671],
[0.354973942041397	,0.209328874945641	,0.696416974067688],
[0.331118136644363	,0.260487824678421	,0.690889656543732],
[0.307262301445007	,0.311646789312363	,0.680652499198914],
[0.283406496047974	,0.362805783748627	,0.665488123893738],
[0.259550690650940	,0.413964748382568	,0.645049095153809],
[0.235694855451584	,0.465123713016510	,0.618813037872315],
[0.211839050054550	,0.516282677650452	,0.586001813411713],
[0.187983229756355	,0.567441642284393	,0.545430064201355],
[0.351526618003845	,0.0831551253795624	,0.724110424518585],
[0.303814977407455	,0.185473054647446	,0.726717174053192],
[0.279959172010422	,0.236632019281387	,0.721422076225281],
[0.256103336811066	,0.287790983915329	,0.711624264717102],
[0.208391711115837	,0.390108913183212	,0.677650034427643],
[0.184535890817642	,0.441267877817154	,0.652725696563721],
[0.160680085420609	,0.492426842451096	,0.621706843376160],
[0.228800207376480	,0.212776198983192	,0.746457278728485],
[0.204944387078285	,0.263935178518295	,0.736992359161377],
[0.157232746481895	,0.366253107786179	,0.704243004322052],
[0.109521113336086	,0.468571037054062	,0.650590479373932],
[0.0856653004884720	,0.519729971885681	,0.614299476146698],
[0.533054530620575	,0.365260809659958	,0.612185955047607],
[0.506813108921051	,0.421535670757294	,0.598149120807648],
[0.607986688613892	,0.0576451122760773	,0.647582828998566],
[0.581745266914368	,0.113919973373413	,0.664024770259857],
[0.555503845214844	,0.170194834470749	,0.674376308917999],
[0.529262483119965	,0.226469695568085	,0.678915977478027],
[0.503021061420441	,0.282744556665421	,0.677760660648346],
[0.476779669523239	,0.339019417762756	,0.670880913734436],
[0.450538277626038	,0.395294278860092	,0.658097207546234],
[0.398055464029312	,0.507844030857086	,0.613172531127930],
[0.551711797714233	,0.0314037129282951	,0.697826683521271],
[0.525470376014710	,0.0876785740256310	,0.713111162185669],
[0.499229013919830	,0.143953442573547	,0.722760021686554],
[0.472987592220306	,0.200228303670883	,0.726997673511505],
[0.446746200323105	,0.256503164768219	,0.725918829441071],
[0.420504808425903	,0.312778025865555	,0.719499707221985],
[0.394263416528702	,0.369052886962891	,0.707594931125641],
[0.368021994829178	,0.425327748060226	,0.689920544624329],
[0.341780602931976	,0.481602609157562	,0.666017413139343],
[0.315539211034775	,0.537877440452576	,0.635182678699493],
[0.289297819137573	,0.594152331352234	,0.596342206001282],
[0.263056427240372	,0.650427222251892	,0.547795593738556],
[0.442954152822495	,0.117712035775185	,0.763065993785858],
[0.416712731122971	,0.173986896872520	,0.767081022262573],
[0.390471339225769	,0.230261757969856	,0.766058683395386],
[0.364229947328568	,0.286536633968353	,0.759978652000427],
[0.337988555431366	,0.342811495065689	,0.748717725276947],
[0.311747133731842	,0.399086356163025	,0.732036888599396],
[0.285505741834641	,0.455361217260361	,0.709554016590118],
[0.259264349937439	,0.511636078357697	,0.680694341659546],
[0.233022943139076	,0.567910909652710	,0.644602000713348],
[0.386679291725159	,0.0914706364274025	,0.796521484851837],
[0.334196478128433	,0.204020366072655	,0.799388885498047],
[0.229230880737305	,0.429119795560837	,0.745415091514587],
[0.202989488840103	,0.485394656658173	,0.717998266220093],
[0.581514060497284	,0.398466348648071	,0.667839229106903],
[0.634631216526032	,0.124276332557201	,0.724390685558319],
[0.577377259731293	,0.247057855129242	,0.740635633468628],
[0.548750281333923	,0.308448612689972	,0.739375293254852],
[0.520123302936554	,0.369839370250702	,0.731870114803314],
[0.573240458965302	,0.0956493541598320	,0.777939438819885],
[0.515986502170563	,0.218430876731873	,0.793088316917419],
[0.487359493970871	,0.279821634292603	,0.791911482810974],
[0.372851580381393	,0.525384664535523	,0.726564407348633],
[0.315597623586655	,0.648166179656982	,0.650555133819580],
[0.311460822820663	,0.496757686138153	,0.774058878421783],
[0.282833844423294	,0.558148443698883	,0.742575645446777],
[0.254206866025925	,0.619539201259613	,0.703202128410339],
[0.250070065259933	,0.468130707740784	,0.813180088996887],
[0.221443071961403	,0.529521465301514	,0.783270835876465],
[0.634455084800720	,0.595697760581970	,0.590981900691986],
[0.594479441642761	,0.334152668714523	,0.800989925861359],
[0.652023017406464	,0.0371134765446186	,0.824704289436340],
[0.621010482311249	,0.103620134294033	,0.842767775058746],
[0.585516393184662	,0.00610091490671039	,0.873946189880371],
[0.554503798484802	,0.0726075693964958	,0.891011953353882],
[0.821678161621094	,0.531673669815064	,0.570630013942719],
[0.788280010223389	,0.603296220302582	,0.545984208583832],
[0.783453762531281	,0.426652997732163	,0.698296487331390],
[0.750055611133575	,0.498275548219681	,0.687450766563416],
[0.716657459735870	,0.569898128509522	,0.667134702205658],
[0.683259308338165	,0.641520678997040	,0.636442005634308],
[0.711831212043762	,0.393254846334457	,0.788731575012207],
[0.678433060646057	,0.464877396821976	,0.779145717620850],
[0.645034909248352	,0.536499977111816	,0.761280655860901],
[0.773801207542419	,0.0733665078878403	,0.824196279048920],
[0.740403056144714	,0.144989058375359	,0.845122456550598],
[0.673606753349304	,0.288234174251556	,0.864074945449829],
[0.640208661556244	,0.359856724739075	,0.862604498863220],
[0.606810510158539	,0.431479275226593	,0.853848457336426],
[0.702178657054901	,0.0399683602154255	,0.888143062591553],
[0.668780505657196	,0.111590914428234	,0.907596051692963],
[0.635382354259491	,0.183213457465172	,0.919876396656036],
[0.601984202861786	,0.254836022853851	,0.925269722938538],
[0.568586051464081	,0.326458573341370	,0.923896729946137],
[0.630556106567383	,0.00657021580263972	,0.941172838211060],
[0.597157955169678	,0.0781927704811096	,0.959551334381104],
[0.525535404682159	,0.0447946228086948	,1.00262475013733],
[0.957107901573181	,0.605434119701386	,0.436741113662720],
[0.880369424819946	,0.569650411605835	,0.611389279365540],
[0.844585716724396	,0.646388828754425	,0.584983110427856],
[0.839414715766907	,0.457128226757050	,0.748174786567688],
[0.803631007671356	,0.533866643905640	,0.736554384231567],
[0.732063531875610	,0.687343537807465	,0.681902170181274],
[0.726892530918121	,0.498082935810089	,0.834798991680145],
[0.691108822822571	,0.574821412563324	,0.815657854080200],
[0.655325114727020	,0.651559829711914	,0.786999106407166],
[0.583757638931274	,0.805036723613739	,0.696054339408875],
[0.793289005756378	,0.155345410108566	,0.905488312244415],
[0.757505297660828	,0.232083871960640	,0.919604063034058],
[0.721721529960632	,0.308822304010391	,0.925794541835785],
[0.685937821865082	,0.385560750961304	,0.924219131469727],
[0.650154113769531	,0.462299197912216	,0.914837598800659],
[0.614370346069336	,0.539037644863129	,0.897405266761780],
[0.507019162178040	,0.769253015518189	,0.790272176265717],
[0.752334296703339	,0.0428232438862324	,0.951581895351410],
[0.716550529003143	,0.119561694562435	,0.972424328327179],
[0.680766820907593	,0.196300134062767	,0.985581815242767],
[0.644983112812042	,0.273038595914841	,0.991360425949097],
[0.609199345111847	,0.349777042865753	,0.989889323711395],
[0.573415637016296	,0.426515489816666	,0.981135964393616],
[0.639812111854553	,0.0837779641151428	,1.02809071540833],
[0.317758560180664	,0.774424016475678	,0.879002690315247],
[0.939060747623444	,0.607627093791962	,0.652148544788361],
[0.857206404209137	,0.569457769393921	,0.785658001899719],
[0.693497717380524	,0.493119150400162	,0.975826799869537],
[0.655328392982483	,0.574973523616791	,0.957232296466827]







	
	
	
]
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    #create pcl from points
    scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
    #publish    
    rospy.loginfo("happily publishing sample pointcloud.. !")
    pcl_pub.publish(scaled_polygon_pcl)