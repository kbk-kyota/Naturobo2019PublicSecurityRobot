# Naturobo2019PublicSecurityRobot
基本的にnhk2019_main_rosをパクってる

公安ロボットは機構として、
スリッパ回収(モータによる位置制御)
昇降するためのロック解除(エアシリンダーを使った電磁弁駆動制御)
足回り
に分かれる(はず)

全体の流れとしては、
スタートしてから、スリッパ回収機構をセットして工場ゾーンからスリッパがはいったケースを回収し、昇降機構のロックを解除して昇降し、スパイロボットにケースに入ったスリッパを渡す
といった感じ
手動制御

ボタンの割り当てはとりあえずで
a...スリッパ回収機構の制御
b...昇降機構
x...受け渡し機構
start...シャットダウン
select...リセット
にした、今後変わる(と思う)

本当に手動機なので、たいそうなことをしてない
スリッパ回収機構のとこは、ケースの位置に合わせて3箇所のポイントに動くようになっている
リセット時はスリッパ回収機構は初期位置に戻るようにしている今までとあんま変わらない
ただ8ビットで動かしたいとこを論理演算して代入するだけ
