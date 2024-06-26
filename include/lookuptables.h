//LOOKUPTABLES

//Corrected values calculated from power measurements obtained 4/4-22 on baxter with spi laser and scanlabs optics SEAA.
//WATT, SETPOINT(0,1023)
#ifdef BAXTER
static const lookuptable correctedLaserPower[] PROGMEM = {
  {0, 56},
  {1, 60},
  {2, 65},
  {3, 69},
  {4, 73},
  {5, 78},
  {6, 82},
  {7, 87},
  {8, 91},
  {9, 95},
  {10, 100},
  {11, 104},
  {12, 108},
  {13, 113},
  {14, 117},
  {15, 121},
  {16, 126},
  {17, 130},
  {18, 134},
  {19, 139},
  {20, 143},
  {21, 147},
  {22, 152},
  {23, 156},
  {24, 160},
  {25, 165},
  {26, 169},
  {27, 173},
  {28, 178},
  {29, 182},
  {30, 187},
  {31, 191},
  {32, 195},
  {33, 200},
  {34, 204},
  {35, 208},
  {36, 213},
  {37, 217},
  {38, 221},
  {39, 226},
  {40, 230},
  {41, 234},
  {42, 239},
  {43, 243},
  {44, 247},
  {45, 252},
  {46, 256},
  {47, 260},
  {48, 265},
  {49, 269},
  {50, 273},
  {51, 278},
  {52, 282},
  {53, 287},
  {54, 291},
  {55, 295},
  {56, 300},
  {57, 304},
  {58, 308},
  {59, 313},
  {60, 317},
  {61, 321},
  {62, 326},
  {63, 330},
  {64, 334},
  {65, 339},
  {66, 343},
  {67, 347},
  {68, 352},
  {69, 356},
  {70, 360},
  {71, 365},
  {72, 369},
  {73, 373},
  {74, 378},
  {75, 382},
  {76, 387},
  {77, 391},
  {78, 395},
  {79, 400},
  {80, 404},
  {81, 408},
  {82, 413},
  {83, 417},
  {84, 421},
  {85, 426},
  {86, 430},
  {87, 434},
  {88, 439},
  {89, 443},
  {90, 447},
  {91, 452},
  {92, 456},
  {93, 460},
  {94, 465},
  {95, 469},
  {96, 473},
  {97, 478},
  {98, 482},
  {99, 487},
  {100, 491},
  {101, 495},
  {102, 500},
  {103, 504},
  {104, 508},
  {105, 513},
  {106, 517},
  {107, 521},
  {108, 526},
  {109, 530},
  {110, 534},
  {111, 539},
  {112, 543},
  {113, 547},
  {114, 552},
  {115, 556},
  {116, 560},
  {117, 565},
  {118, 569},
  {119, 573},
  {120, 578},
  {121, 582},
  {122, 587},
  {123, 591},
  {124, 595},
  {125, 600},
  {126, 604},
  {127, 608},
  {128, 613},
  {129, 617},
  {130, 621},
  {131, 626},
  {132, 630},
  {133, 634},
  {134, 639},
  {135, 643},
  {136, 647},
  {137, 652},
  {138, 656},
  {139, 660},
  {140, 665},
  {141, 669},
  {142, 673},
  {143, 678},
  {144, 682},
  {145, 687},
  {146, 691},
  {147, 695},
  {148, 700},
  {149, 704},
  {150, 708},
  {151, 713},
  {152, 717},
  {153, 721},
  {154, 726},
  {155, 730},
  {156, 734},
  {157, 739},
  {158, 743},
  {159, 747},
  {160, 752},
  {161, 756},
  {162, 760},
  {163, 765},
  {164, 769},
  {165, 773},
  {166, 778},
  {167, 782},
  {168, 787},
  {169, 791},
  {170, 795},
  {171, 800},
  {172, 804},
  {173, 808},
  {174, 813},
  {175, 817},
  {176, 821},
  {177, 826},
  {178, 830},
  {179, 834},
  {180, 839},
  {181, 843},
  {182, 847},
  {183, 852},
  {184, 856},
  {185, 860},
  {186, 865},
  {187, 869},
  {188, 873},
  {189, 878},
  {190, 882},
  {191, 887},
  {192, 891},
  {193, 895},
  {194, 900},
  {195, 904},
  {196, 908},
  {197, 913},
  {198, 917},
  {199, 921},
  {200, 926},
  {201, 930},
  {202, 934},
  {203, 939},
  {204, 943},
  {205, 947},
  {206, 952},
  {207, 956},
  {208, 960},
  {209, 965},
  {210, 969},
  {211, 973},
  {212, 978},
  {213, 982},
  {214, 987},
  {215, 991},
  {216, 995},
  {217, 1000},
  {218, 1004},
  {219, 1008},
  {220, 1013},
  {221, 1017},
  {222, 1021}};
#endif

#ifdef LOOP
static const lookuptable correctedLaserPower[] PROGMEM = {
  {150, 478},
  {151, 481},
  {152, 484},
  {153, 487},
  {154, 490},
  {155, 493},
  {156, 495},
  {157, 498},
  {158, 501},
  {159, 504},
  {160, 507},
  {161, 510},
  {162, 512},
  {163, 515},
  {164, 518},
  {165, 521},
  {166, 524},
  {167, 527},
  {168, 529},
  {169, 532},
  {170, 535},
  {171, 538},
  {172, 541},
  {173, 544},
  {174, 546},
  {175, 549},
  {176, 552},
  {177, 555},
  {178, 558},
  {179, 561},
  {180, 563},
  {181, 566},
  {182, 569},
  {183, 572},
  {184, 575},
  {185, 578},
  {186, 580},
  {187, 583},
  {188, 586},
  {189, 589},
  {190, 592},
  {191, 595},
  {192, 597},
  {193, 600},
  {194, 603},
  {195, 606},
  {196, 609},
  {197, 612},
  {198, 614},
  {199, 617},
  {200,620},
  {201, 623},
  {202, 626},
  {203, 629},
  {204, 631},
  {205, 634},
  {206, 637},
  {207, 640},
  {208, 643},
  {209, 646},
  {210, 648},
  {211, 651},
  {212, 654},
  {213, 657},
  {214, 660},
  {215, 663},
  {216, 665},
  {217, 668},
  {218, 671},
  {219, 674},
  {220, 677},
  {221, 680},
  {222, 682},
  {223, 685},
  {224, 688},
  {225, 691},
  {226, 694},
  {227, 697},
  {228, 699},
  {229, 702},
  {230, 705},
  {231, 708},
  {232, 711},
  {233, 714},
  {234, 716},
  {235, 719},
  {236, 722},
  {237, 725},
  {238, 728},
  {239, 731},
  {240, 733},
  {241, 736},
  {242, 739},
  {243, 742},
  {244, 745},
  {245, 748},
  {246, 750},
  {247, 753},
  {248, 756},
  {249, 759},
  {250, 762},
  {251, 765},
  {252, 767},
  {253, 770},
  {254, 773},
  {255, 776},
  {256, 779},
  {257, 782},
  {258, 784},
  {259, 787},
  {260, 790},
  {261, 793},
  {262, 796},
  {263, 799},
  {264, 801},
  {265, 804},
  {266, 807},
  {267, 810},
  {268, 813},
  {269, 816},
  {270, 818},
  {271, 821},
  {272, 824},
  {273, 827},
  {274, 830},
  {275, 833},
  {276, 835},
  {277, 838},
  {278, 841},
  {279, 844},
  {280, 847},
  {281, 850},
  {282, 852},
  {283, 855},
  {284, 858},
  {285, 861},
  {286, 864},
  {287, 867},
  {288, 869},
  {289, 872},
  {290, 875},
  {291, 878},
  {292, 881},
  {293, 884},
  {294, 886},
  {295, 889},
  {296, 892},
  {297, 895},
  {298, 898},
  {299, 901},
  {300, 903}};
#endif
