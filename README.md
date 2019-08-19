# Naturobo2019PublicSecurityRobot
基本的にnhk2019_main_rosを流用している

公安ロボットは機構として、
スリッパ回収(モータによる位置制御)
昇降するためのロック解除(エアシリンダーを使った電磁弁駆動制御)
スリッパをスパイロボットに送る(エアシリンダーを使った電磁弁駆動制御)
足回り
に分かれる(はず)

全体の流れとしては、
スタートしてから、スリッパ回収機構をセットして工場ゾーンからスリッパがはいったケースを回収し、昇降機構のロックを解除して昇降し、スパイロボットにケースに入ったスリッパを渡すために展開し、展開されたのを戻し、スリッパ回収機構を次の位置にセットして再び工場ゾーンへ...
といった感じである
一部自動制御が入ってはいるもののほぼ手動制御である

srcのnr_can.cppとnr_main_semiauto.cppでは
スリッパ回収機構のセットをset_collectingsulipper
昇降するためのロック解除をelavate_case
ロックをdescent_sase
スリッパをスパイロボットに送るのをsend_slipper
としている

二つのエアシリの駆動は一つの基板で行う
