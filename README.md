


### serial commands


|command|size|data  |direction|description                                               |
|0x01   |0   |-     |PC->PIC  |演奏開始                                                  |
|0x02   |1   |size  |PIC->PC  |データ要求指示。size 分を要求                                                       |
|0x03   |N   |data  |PC->PIC  |データ転送。N に転送するサイズを格納。上記 size 以下になる。またNが0の場合はデータなしで演奏終了|
