
# The following data come from one reading of the "scan" topic.
# I have converted this into a Python "class", so it will mimic what 
# you'd get if you subscribed to the "scan" topic. 
class create_scan_data:
	def __init__(self):
		self.angle_min       = -0.521567881107
		self.angle_max       = 0.524276316166
		self.angle_increment = 0.00163668883033
		self.time_increment  = 0.0
		self.scan_time       = 0.0329999998212
		self.range_min       = 0.449999988079
		self.range_max       = 10.0
		self.ranges          = [1.6017869710922241, 1.5993984937667847, 1.5970163345336914, 1.5946406126022339, 1.5899080038070679, 1.587551236152649, 1.5852009057998657, 1.5828567743301392, 1.5805190801620483, 1.5781877040863037, 1.573543667793274, 1.5712313652038574, 1.5689250230789185, 1.5666249990463257, 1.5643314123153687, 1.5597628355026245, 1.557487964630127, 1.555219292640686, 1.5529569387435913, 1.5507009029388428, 1.5484508275985718, 1.543969750404358, 1.541738510131836, 1.5395134687423706, 1.537294626235962, 1.5350819826126099, 1.532875418663025, 1.5306751728057861, 1.5262932777404785, 1.5241113901138306, 1.5219359397888184, 1.5197663307189941, 1.5176030397415161, 1.5154459476470947, 1.5132948160171509, 1.509011149406433, 1.5068786144256592, 1.5047519207000732, 1.5026315450668335, 1.5005172491073608, 1.4984089136123657, 1.4963067770004272, 1.4942106008529663, 1.4900367259979248, 1.4879589080810547, 1.4858869314193726, 1.4838212728500366, 1.4817615747451782, 1.4797078371047974, 1.4776601791381836, 1.475618600845337, 1.4735828638076782, 1.4695298671722412, 1.4675122499465942, 1.4655007123947144, 1.463495135307312, 1.4614955186843872, 1.4595019817352295, 1.4575142860412598, 1.4555326700210571, 1.453959345817566, 1.4563707113265991, 1.4587963819503784, 1.463692307472229, 1.4661625623703003, 1.468647837638855, 1.4711482524871826, 1.4736639261245728, 1.4761948585510254, 1.4787415266036987, 1.4813034534454346, 1.4838812351226807, 1.4864749908447266, 1.4890846014022827, 1.4917103052139282, 1.4943522214889526, 1.4996850490570068, 1.5023763179779053, 1.5050842761993408, 1.507809042930603, 1.5105509757995605, 1.5133097171783447, 1.5160858631134033, 1.5188794136047363, 1.5216903686523438, 1.5245190858840942, 1.5273653268814087, 1.5302298069000244, 1.5331121683120728, 1.536012887954712, 1.5389318466186523, 1.5418692827224731, 1.5448254346847534, 1.5478003025054932, 1.550794243812561, 1.5568393468856812, 1.5598909854888916, 1.5629620552062988, 1.5660529136657715, 1.5691637992858887, 1.5722944736480713, 1.575445532798767, 1.5786168575286865, 1.5818088054656982, 1.5850214958190918, 1.5882549285888672, 1.591509461402893, 1.5947853326797485, 1.5980823040008545, 1.6014010906219482, 1.6047414541244507, 1.60810387134552, 1.6114884614944458, 1.614895224571228, 1.6183245182037354, 1.6217764616012573, 1.625251293182373, 1.628749132156372, 1.632270336151123, 1.635814905166626, 1.63938307762146, 1.642975091934204, 1.6465911865234375, 1.6502315998077393, 1.6538965702056885, 1.6575859785079956, 1.6613004207611084, 1.6650398969650269, 1.66880464553833, 1.6725950241088867, 1.6764111518859863, 1.680253267288208, 1.6841216087341309, 1.688016414642334, 1.691937804222107, 1.6958861351013184, 1.6998616456985474, 1.703864574432373, 1.707895040512085, 1.7119535207748413, 1.7160398960113525, 1.720154881477356, 1.724298357963562, 1.7284706830978394, 1.7326722145080566, 1.736903190612793, 1.7411638498306274, 1.7454544305801392, 1.7497752904891968, 1.7541265487670898, 1.758508563041687, 1.762921690940857, 1.7673660516738892, 1.7718422412872314, 1.7763501405715942, 1.7808904647827148, 1.7854632139205933, 1.7900686264038086, 1.7947074174880981, 1.799379587173462, 1.804085373878479, 1.8088253736495972, 1.8135995864868164, 1.8184086084365845, 1.82325279712677, 1.8281322717666626, 1.8330473899841309, float('nan'), 1.8379987478256226, 1.8429863452911377, 1.8480106592178345, 1.8530722856521606, 1.8581713438034058, 1.8633081912994385, 1.8684831857681274, 1.8736969232559204, 1.8789494037628174, 1.8842413425445557, 1.8895729780197144, 1.8949447870254517, 1.9003571271896362, 1.9058103561401367, 1.9113048315048218, 1.9168411493301392, 1.922419786453247, 1.9280407428741455, 1.9337048530578613, 1.9394124746322632, 1.9451638460159302, 1.9509596824645996, float('nan'), 1.9568002223968506, 1.9626860618591309, 1.9686177968978882, 1.9745956659317017, 1.98062002658844, 1.9866917133331299, 1.9928109645843506, 1.9989783763885498, 2.0051944255828857, 2.0114598274230957, 2.017774820327759, 2.024139881134033, 2.0305559635162354, 2.0370230674743652, 2.0435423851013184, 2.0501136779785156, float('nan'), 2.0567383766174316, 2.0634162425994873, 2.070148468017578, 2.076935052871704, 2.0837771892547607, 2.0906753540039062, 2.0976297855377197, 2.1046414375305176, 2.111711025238037, 2.1188390254974365, 2.126025915145874, 2.133272647857666, 2.14057993888855, 2.1479480266571045, float('nan'), 2.1553781032562256, 2.162870407104492, 2.170426368713379, 2.1780459880828857, 2.185730218887329, 2.1934800148010254, 2.2012956142425537, 2.2091784477233887, 2.2171287536621094, 2.2251477241516113, 2.233236074447632, 2.241394281387329, 2.2496235370635986, float('nan'), 2.2579245567321777, 2.266298294067383, 2.274745464324951, 2.283267021179199, 2.2918639183044434, 2.300536632537842, 2.3092870712280273, 2.318115234375, 2.3270223140716553, 2.3360095024108887, 2.3450775146484375, 2.3542277812957764, float('nan'), 2.3634607791900635, 2.3727781772613525, 2.3821802139282227, 2.3916687965393066, 2.4012441635131836, 2.4109082221984863, 2.420661687850952, 2.4305055141448975, 2.440441608428955, 2.450470209121704, 2.4605932235717773, float('nan'), 2.470811367034912, 2.481126308441162, 2.491539478302002, 2.50205135345459, 2.5126638412475586, 2.5233781337738037, 2.534195899963379, 2.5451180934906006, 2.5561463832855225, 2.567282199859619, float('nan'), 2.578526735305786, 2.589881658554077, 2.601348400115967, 2.612928628921509, 2.6246237754821777, 2.6364352703094482, 2.6483654975891113, 2.6604154109954834, 2.672586441040039, 2.6848812103271484, 2.6973013877868652, float('nan'), 2.7098476886749268, 2.7225229740142822, 2.735328435897827, 2.7482666969299316, float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), float('nan'), 2.110891342163086, 2.105895519256592, 2.10093355178833, 2.0960049629211426, 2.0911099910736084, 2.086247682571411, 2.08141827583313, 2.0766215324401855, 2.07185697555542, 2.067124128341675, 2.06242299079895, 2.057753324508667, 2.053114891052246, 2.0485072135925293, 2.0439300537109375, 2.0393831729888916, 2.0348665714263916, 2.0303797721862793, 2.0259225368499756, 2.0214946269989014, 2.0170958042144775, 2.012726068496704, 2.0083847045898438, 2.0040717124938965, 1.9997872114181519, 1.9955304861068726, 1.9913016557693481, 1.9871001243591309, 1.9829258918762207, float('nan'), 1.978778600692749, 1.9746581315994263, 1.9705644845962524, 1.9664971828460693, 1.9624559879302979, 1.9584407806396484, 1.954451322555542, 1.950487494468689, 1.9465491771697998, 1.9426356554031372, 1.9387474060058594, 1.9348838329315186, 1.9310448169708252, 1.9272300004959106, 1.923439621925354, 1.919673204421997, 1.9122116565704346, 1.9085159301757812, 1.904843807220459, 1.9011945724487305, 1.8975681066513062, 1.8939646482467651, 1.8903834819793701, 1.8868247270584106, 1.8832882642745972, 1.8797738552093506, 1.8762812614440918, 1.8728102445602417, 1.8693609237670898, 1.865932822227478, 1.8625259399414062, 1.8591402769088745, 1.855775237083435, 1.852431058883667, 1.8491075038909912, 1.8458043336868286, 1.8425214290618896, 1.8392585515975952, 1.8360158205032349, 1.8327927589416504, 1.8295894861221313, 1.826405644416809, 1.823241114616394, 1.8200958967208862, 1.816969871520996, 1.8138625621795654, 1.810774326324463, 1.8077046871185303, 1.8046534061431885, 1.8016207218170166, 1.795609951019287, 1.7926315069198608, 1.7896710634231567, 1.7867282629013062, 1.783803105354309, 1.7808955907821655, 1.7780052423477173, 1.7751322984695435, 1.7722764015197754, 1.769437313079834, 1.7666151523590088, 1.7638099193572998, 1.7610210180282593, 1.758249044418335, 1.7554931640625, 1.752753496170044, 1.7473227977752686, 1.744631290435791, 1.7419557571411133, 1.7392957210540771, 1.7366514205932617, 1.734022617340088, 1.7314090728759766, 1.7288107872009277, 1.7262277603149414, 1.723659873008728, 1.721106767654419, 1.7185685634613037, 1.713536262512207, 1.7110419273376465, 1.7085621356964111, 1.7060965299606323, 1.7036452293395996, 1.7012081146240234, 1.6987850666046143, 1.696375846862793, 1.6939804553985596, 1.6915990114212036, 1.686876654624939, 1.6845358610153198, 1.6822082996368408, 1.679894208908081, 1.6775932312011719, 1.6753053665161133, 1.6730304956436157, 1.6707686185836792, 1.6662832498550415, 1.6640595197677612, 1.661848545074463, 1.6596500873565674, 1.6574639081954956, 1.655290126800537, 1.6531286239624023, 1.6509792804718018, 1.646716833114624, 1.6446034908294678, 1.6425020694732666, 1.640412449836731, 1.6383345127105713, 1.6362680196762085, 1.6342133283615112, 1.6301380395889282, 1.628117322921753, 1.626107931137085, 1.6241097450256348, 1.6221226453781128, 1.6201465129852295, 1.6162270307540894, 1.6142836809158325, 1.6123509407043457, 1.6104289293289185, 1.6085172891616821, 1.6066166162490845, 1.6028459072113037, 1.6009763479232788, 1.5991168022155762, 1.597267508506775, 1.5954283475875854, 1.5917800664901733, 1.5899710655212402, 1.5881717205047607, 1.5863823890686035, 1.5846025943756104, 1.5828325748443604, 1.5793213844299316, 1.5775799751281738, 1.5758482217788696, 1.5741256475448608, 1.5724124908447266, 1.5707085132598877]
		self.intensities  = []
	
# We will use the class we created above to assign values to the "scan_data" variable.		
scan_data = create_scan_data()
		
# Just for fun, let's print the value of the maximum angle:
print(scan_data.angle_max)

# Let's also print the number of values in the "ranges" list:
print(len(scan_data.ranges))

# *********************************************
# Now, for your homework, you are asked to 
# write some Python code to do the following:
#
# 1) What is the sweep angle of the scanner, 
#    in degrees?
#    A sweep angle of 180 degrees would indicate
#    that the robot can see everything in front 
#    of it.  A sweep angle of 90 degrees would 
#    indicate that the robot can see 45 degrees
#    to the left and 45 degrees to the right.
# 
# 2) Plot the locations of where the scanner 
#    thinks that there are obstacles.
#    Assume that your sensor is located at the
#    point (x,y) = (0,0).
#    Use matplotlib to create your plot.
#
# 3) There appears to be a gap between obstacles.
#    Using the ranges data, calculate the size of
#    the gap (in meters).
#
# 4) What is the angle that the turtlebot must 
#    rotate (in radians) to face the middle point 
#    of the gap between obstacles?  
#    The idea is that, if your turtlebot goes in
#    that direction, it is likely to be able to 
#    travel between the obstacles.
