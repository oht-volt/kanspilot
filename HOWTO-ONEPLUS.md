원플(Oneplus 3t)에서 오픈파일럿 이식하려면?
------
1. /data 디렉토리에서 아래 명령을 수행하여 oneplus 브렌치를 클론합니다.
```
cd /data/ && mv openpilot openpilot-backup;
git clone --depth 1 recurse-submodules https://github.com/kans-ky/openpilot --branch oneplus
```

2. 클론이 완료되면 바로 재부팅하지 마시고, 아래 명령 실행합니다
```
chmod 700 /data/openpilot/unix.sh;
chmod 700 /data/openpilot/scripts/oneplus_update_neos.sh;
chmod 700 /data/openpilot/selfdrive/hardware/eon/updater
cd /data/openpilot/ && ./unix.sh; 
cd /data/openpilot/scripts/ && ./oneplus_update_neos.sh
```

3. 원플용 Neos(recovery, system 이미지)가 다운로드되면 자동으로 부팅됩니다.
   recovery이미지는 금방 다운도드되지만 시스템이미지는 시간이 좀 걸립니다.
   다운로드가 완료되면 자동으로 시스템이 재부팅되고, neos업데이트 깡통로봇이 나옵니다.
   업데이트가 완료되면 자동으로 패스트모드로 부팅됩니다.

4. fastboot 모드에서 볼륨 상하버튼으로 리커버리모드 선택한 뒤 파워버튼을 눌러 리커버리 모드로 진입합니다. 

5. 리커버리 모드에서 차례대로 apply update` -> `Choose from emulated` -> `0/` -> `update.zip` -> `Reboot system now` 선택합니다.

 
6. 이제 원플에서도 오픈파일럿이 부팅됩니다. 터치버튼이 먹히지 않으으로 강제로 리부팅시킵니다.
   부팅후 와이파이에 접속하시고 ssh로 접속, 본인이 부여받은 id_rsa파일과 연결시켜 이온과 접속해서 각자 환경에 맞게 파일들을 수정하시면 됩니다.
