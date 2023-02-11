// 載入arduino內建的Servo功能
#include <Servo.h>
// 建立Servo物件 以控制伺服馬達
Servo myservol;
Servo myservor;
//給定光敏電阻跨壓值輸入pin
byte lr=A0;  byte rr=A2;
void setup() {  //setup function開始
//傳輸協定之包爾值
  Serial.begin(9600);
//如果有接上螢幕 對螢幕輸出
  Serial.print("Hello! Press N to turn OFF the LED \n");
/設定開關pin
  pinMode(4,INPUT);
//設定光敏電阻跨壓值輸入pin
  pinMode(lr,INPUT);pinMode(rr,INPUT);
//伺服馬達設定輸出pin
  myservol.attach(5);
  myservor.attach(6);
//讓伺服馬達輸出 0 degree 以歸零
  myservol.write(0);
  myservor.write(0);
//讓馬達跑一下
  delay(100);
} 
void loop() {  //loop function 開始
//設定光敏電阻讀取參數
  float lv; float rv; 
//設定誤差參數
  float e[2]={0,0}; float temp;
//設定輸出參數
  float U=0; float dU=0;
//*******************************************************************  //因為直接使用loop作為迴圈的話 每一次都會回復粗使值設定狀態 所以改由while (true) 讓控制器一直運行
  while (true) {
//數位開關
    boolean button;     //開關參數
    button=digitalRead(4);//讀取是高電位/低店位
    if (button==0) {      //如果低電位則關(使用無限迴圈) 並歸零
      Serial.print("trun off\n");
      myservol.write(0);
      myservor.write(0);
      delay(100);
      while (true) {
         button=digitalRead(4);//變成高電位break無限迴圈
         delay(1000);
         if (button==1) {break;}
      }
    }
    Serial.print("trun on\n");
//*******************************************************************    
//初步讀取光敏電阻跨壓值
    lv=analogRead(lr); rv=analogRead(rr);
//對光敏電阻跨壓值進行處理 unit [1/1023 V]
    lv=sqrt(678-lv)*26;   rv=(678-rv);
//誤差計算
    e[0]=lv-rv;          e[1]=e[0]-temp;        temp=e[0];
// Fuzzy開始
//******************************************************************* 
// 自動讀取誤差變數之size [m_e,n_e]
    int n_e=1; int m_e=sizeof(e)/sizeof(float)/n_e;
//*******************************************************************    // 設定誤差之membership
    // e
    float miZM_x= 0     ;float upZM_x=50;
    float buPS_x= miZM_x;float upPS_x=50;
    float buZM_x=-upZM_x;
    float upNS_x=-buPS_x;float buNS_x=-upPS_x;
    // de
    float miZM_dx= 0      ;float upZM_dx=30;
    float buPS_dx= miZM_dx;float upPS_dx=30;
    float buZM_dx=-upZM_dx;
    float upNS_dx=-buPS_dx;float buNS_dx=-upPS_dx;  
    // 設定誤差membership之運算參數
    float buNS[m_e];float upNS[m_e];
    float buZM[m_e];float miZM[m_e];float upZM[m_e];
    float buPS[m_e];float upPS[m_e];
    // e
    buNS[0]=buNS_x;upNS[0]=upNS_x;
    buZM[0]=buZM_x;miZM[0]=miZM_x;upZM[0]=upZM_x;
    buPS[0]=buPS_x;upPS[0]=upPS_x;
    // de
    buNS[1]=buNS_dx;upNS[1]=upNS_dx;
    buZM[1]=buZM_dx;miZM[1]=miZM_dx;upZM[1]=upZM_dx;
    buPS[1]=buPS_dx;upPS[1]=upPS_dx;
//*******************************************************************
// 設定 Force membership
    float Null[]={0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    };
    float F_NS[]={1.000,0.875,0.750,0.625,0.500,0.375,0.250,0.125,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    };
    float F_ZM[]={0    ,0.125,0.250,0.375,0.500,0.625,0.750,0.875,1.000,0.875,0.750,0.625,0.500,0.375,0.250,0.125,0    };
    float F_PS[]={0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0    ,0.125,0.250,0.375,0.500,0.625,0.750,0.875,1.000};
    // 自動讀取force membership之size
    int m_t=9;  int n_t=sizeof(Null)/sizeof(float);
    // 設定force membership之運算參數 也就是Rule base
    float tF[m_t][n_t];
    for (int i=0;i<m_t;i++){
        for (int j=0;j<n_t;j++){
          tF[i][j]=Null[j];
          if (i==1){tF[i][j]=F_PS[j];}
          if (i==3){tF[i][j]=F_PS[j];}if (i==4){tF[i][j]=F_ZM[j];}if(i==5){tF[i][j]=F_NS[j];}
          if (i==7){tF[i][j]=F_NS[j];}
        }
    }
//*******************************************************************
// Fuzzification
// 設定歸屬運算矩陣 其中 mhb 為個別歸屬membership變數之size ; m_e 為誤差狀態之size
    int mhb=sqrt(m_t);
    float mht[m_e][mhb];
    // 誤差之歸屬值運算
    for (int i=0;i<m_e;i++) {
        float mh[mhb]={0,0,0};
        //*NS*/******************************************************
        if (e[i]<=buNS[i]) {mh[2]=1;}
        if ((e[i]>=buNS[i])&(e[i]<=upNS[i])) {mh[2]=(upNS[i]-e[i])/(upNS[i]-buNS[i]);}
        //*Z*/*******************************************************
        if ((e[i]>=buZM[i])&(e[i]<=miZM[i])) {mh[1]=(e[i]-buZM[i])/(miZM[i]-buZM[i]);}
        if ((e[i]>=miZM[i])&(e[i]<=upZM[i])){mh[1]=(upZM[i]-e[i])/(upZM[i]-miZM[i]);}
        //*PS*/*******************************************************
        if ((e[i]>=buPS[i])&(e[i]<=upPS[i])) {mh[0]=(e[i]-buPS[i])/(upPS[i]-buPS[i]);}
        if (e[i]>=upPS[i]) {mh[0]=1;}
        // 歸屬值儲存
        for (int k=0;k<mhb;k++){
          mht[i][k]=mh[k];
        }
    }
    //偵錯用螢幕
    //不知為何必須凱起此功能 才會運作正常
    for (int k=0;k<mhb;k++){
      Serial.print("mht[0][");Serial.print(k);;Serial.print("]="); Serial.print(mht[0][k]); Serial.print("\t");
    }Serial.print("\n");
    for (int k=0;k<mhb;k++){
      Serial.print("mht[1][");Serial.print(k);;Serial.print("]="); Serial.print(mht[1][k]); Serial.print("\t");
    }Serial.print("\n");
//*******************************************************************    // Fuzzy Logic & Computation
    // 設定輸出參數
    //其中 n_t 為force membership之size; m_e 為誤差狀態之size
    float u[m_e/2][n_t+1]; //不知為何必須n_t+1才會運作正常
    // 設定運算參數
    int s=0; float M[m_t];
    // 邏輯運算
    // 其中 mhb 為個別歸屬membership變數之size
    // 其中 tF 為 Rule base
    for (int vol=0;vol<m_e/2;vol++) {
        for (int j=0;j<mhb;j++) {
            for (int i=0;i<mhb;i++) {
                M[s]=min(mht[vol][i],mht[vol+1][j]);
                for (int v=0;v<n_t;v++) {
                    u[vol][v]=max(u[vol][v],min(M[s],tF[s][v]));
                }
                s=s+1;
            }
        }
    }
//*******************************************************************    // Defuzzilization 
    // unit of force 1[degree]
    float w=1;
    // unit force begin at  
    int sign=-(n_t-1)/2;
    // 解模糊化
    for (int vol=0;vol<m_e/2;vol++) {
        // 設定運算參數 以及歸零
        float mag=0; float weight=0; float moment=0;
        // 計算
        for (int i=0;i<n_t;i++) {
          mag=sign+i;
          weight=weight+u[vol][i];
          moment=moment+w*mag*u[vol][i];
        }
        // 防止weight==0 使得運算爆掉
        if (weight==0) {
          dU=0;
        }
        // 正常輸出變化量
        else {
          dU=moment/weight;
        }
    }
//*******************************************************************    // Fuzzy Control 結束
// 輸出加總
    U=U+dU;
// 避免角度超出工作範圍 1~179[degree]
    if (U<1) {U=1;}  if (U>179) {U=179;}
// 傳輸指令給伺服馬達
    myservol.write(U);
    myservor.write(U);
// 讓馬達跑一下
    delay(70);
// 偵錯用螢幕
    Serial.print("dU="); Serial.print(dU); Serial.print("\n");
    Serial.print("U="); Serial.print(U); Serial.print("\n");
    Serial.print("\n\n");
  }
}
