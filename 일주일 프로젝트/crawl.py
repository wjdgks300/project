import pandas as pd
import requests
from tqdm import tqdm
import lxml.html
import numpy as np

# 첫회가 무엇인지 받아오기
def first():
    req_urla = "https://www.dhlottery.co.kr/gameResult.do?method=byWin"
    req_lottoa = requests.get(req_urla)
    html = req_lottoa.text
    root = lxml.html.fromstring(html)
    num = root.xpath('//*[@id="article"]/div[2]/div/div[2]/h4/strong')
    thisnum = num[0].text
    print(thisnum.strip("회"))
    return thisnum


def getLottoWinInfo(startRound, endRound):
    drwtNo1 = []
    drwtNo2 = []
    drwtNo3 = []
    drwtNo4 = []
    drwtNo5 = []
    drwtNo6 = []
    bnusNo = []
    totSellamnt = []
    drwNoDate = []
    firstAccumamnt = []
    firstPrzwnerCo = []
    firstWinamnt = []
    roundNo = []
    for i in tqdm(range(startRound, endRound+1, 1)): # i = 1
        req_url = "https://www.dhlottery.co.kr/common.do?method=getLottoNumber&drwNo=" + str(i)
        req_lotto = requests.get(req_url)
        lottoNo = req_lotto.json()
        drwtNo1.append(lottoNo['drwtNo1'])
        drwtNo2.append(lottoNo['drwtNo2'])
        drwtNo3.append(lottoNo['drwtNo3'])
        drwtNo4.append(lottoNo['drwtNo4'])
        drwtNo5.append(lottoNo['drwtNo5'])
        drwtNo6.append(lottoNo['drwtNo6'])
        bnusNo.append(lottoNo['bnusNo'])
        roundNo.append(i)
        totSellamnt.append(lottoNo['totSellamnt'])
        drwNoDate.append(lottoNo['drwNoDate'])
        firstAccumamnt.append(lottoNo['firstAccumamnt'])
        firstPrzwnerCo.append(lottoNo['firstPrzwnerCo'])
        firstWinamnt.append(lottoNo['firstWinamnt'])
        lotto_dict = {"추첨일":drwNoDate, "회차":roundNo, "Num1":drwtNo1, "Num2":drwtNo2, "Num3":drwtNo3, "Num4":drwtNo4, "Num5":drwtNo5, "Num6":drwtNo6, "bnsNum":bnusNo, "총판매금액":totSellamnt, "총1등당첨금":firstAccumamnt, "1등당첨인원":firstPrzwnerCo, "1등수령액":firstWinamnt}
        lotto_df = pd.DataFrame(lotto_dict)
    return lotto_dict, lotto_df
    #%% 로또 회차정보 불러오기

thisnum = first()
endRound =int(thisnum.strip("회"))
startRound = endRound - 50


#lotto_dict, lotto_df = getLottoWinInfo(startRound, endRound)
# CSV 파일로 저장
#lotto_df.to_csv(str(startRound) + '-' + str(endRound) + '_lotto_number.csv', index = False, encoding = 'utf-8-sig')

csv_test = pd.read_csv(str(startRound) + '-' + str(endRound) + '_lotto_number.csv')

proba = np.zeros([1,45])

array_test = csv_test.to_numpy()
# print(proba)

#반복문으로 1~45까지 어떤 수가 나왔는지 count
for j in range(1,len(csv_test.index)):
    for i in range(2,9):
        for k in range(1,45):
            if array_test[j][i] == k:
                proba[0][k] = proba[0][k] + 1

#확률 구하기
percentage = 0
for i in range(1,45):
    percentage += (proba[0][i] / np.sum(proba))
    print(proba[0][i] / np.sum(proba))

#print(percentage)
print(proba)


#print(csv_test.shape)