module hc05_top( 

    input         clk ,
    input         rst , 
//----------------�������ź���----------------------------
    input  [7:0]  data_i ,       //׼�����͵Ĳ����ź�
    input         en_flag_i ,    //�����Ĳ�����Ч�źţ��ߵ�ƽʱ�Ż��data_iͨ���������ⷢ��

    output [7:0]  data_o ,       //׼�����ܵĲ����ź�
    output        en_flag_o,     //�����ź���Ч��־��Ϊ1ʱ��ʾdata_o��Ч

//----------------������ź���----------------------------
    input         rx ,           //���������ն���ӵ��ź� 
    output        tx ,           //���������Ͷ���ӵ��ź�
    output        tx_end

 );



uart_tx uart_tx_inst (
     .sys_clk    (clk        ) ,   //ϵͳʱ��50MHz
     .sys_rst_n  (rst        ) ,   //ȫ�ָ�λ
     .pi_data    (data_i     ) ,   //ģ�������8bit����
     .pi_flag    (en_flag_i  ) ,   //����������Ч��־�ź�,��Ч�ſ�ʼ����
     .tx         (tx         ) ,   //��ת�����1bit����
     .tx_end     (tx_end     )
);




uart_rx uart_rx_inst(
    .sys_clk     (clk         ),   //ϵͳʱ��50MHz
    .sys_rst_n   (rst         ),   //ȫ�ָ�λ
    .rx          (rx          ),   //���ڽ�������
    .po_data     (data_o      ),   //��ת�����8bit����
    .po_flag     (en_flag_o   )    //��ת�����������Ч��־�źţ���Ч��Ϊ1
);



endmodule