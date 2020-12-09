#!/usr/bin/env python

import opcua
import opcua.ua
import rospy
import Queue
import threading
import demo_msgs.srv

'''
#TODO
UP = 1
DOWN =2
'''

class OpcUAServer(object):
    def __init__(self, port=4840, max_list_retries=5):
        self._port = port
        self._started = False
        self._control_busy_variable = None
        self._last_method_result_variable = None
        self._method_calls_counter_variable = None
        self._max_list_retries = max_list_retries

        self._run_demo_method = rospy.ServiceProxy('/iiwa/demo', demo_msgs.srv.OpcuaService)
       
        self._worker_thread = threading.Thread(target=self.worker, name="OpcUAServerWorkerThread")
        self._queue = Queue.Queue()

    def __del__(self):
        self.stop()

    # -----------------------------------------------------------------------------------------------------------------
    # Worker thread
    # -----------------------------------------------------------------------------------------------------------------
    # TODO -> worker thread
    #  -> esegue in ordine da coda synchronized
    #  -> quando parte task setta self._control_busy_variable a true
    #  ->  esegue task
    #  ->  setta self._last_task_result sul risultato
    #  -> finisce. 
    #  ->  incremente self._method_calls_counter_variable per notificare i client che ha fatto una chiamata, poi
    #              -> se altri task in coda, li esegue
    #              -> altrimenti setta self._control_busy_variable a false
    def worker(self):
        while True:
            item = self._queue.get()
            if not self._started:
                rospy.loginfo('[WORKER] Quitting')
                return

            self._control_busy_variable.set_value(True)
            # rospy.loginfo('Calling something')
            try:
                if item == False:
                    return

                if type(item) == list:
                    for i in item:
                        if not self._started:
                            break

                        for j in range(self._max_list_retries):
                            rospy.loginfo('[WORKER] Calling {}'.format(i.__name__))
                            res = i()
                            response = res.response if type(res) != robotnik_msgs.srv.SetLaserModeResponse else res.ret
                            if response:
                                break

                        if not response:
                            rospy.loginfo('[WORKER] Last service in list returned false')
                            break
                else:
                    rospy.loginfo('[WORKER] Calling {}'.format(item.__name__))
                    res = item()
                    response = res.success

                self._last_method_result_variable.set_value(response)
            except Exception as e:
                rospy.loginfo('[WORKER] Exception -> {}'.format(e))
            finally:
                # Increment method counter -> notify opcua client that the call has ended
                self._method_calls_counter_variable.set_value(self._method_calls_counter_variable.get_value() + 1)
                if self._queue.empty():
                    self._control_busy_variable.set_value(False)

    # il deploy di un task viene fatto su wrapper a metodi ua
    # -> se quando chiamo la self._control_busy_variable = true ritorno false, altrimenti ritorno true
    #    -> il valore di ritorno mi dice se il sistema ha capito.
    #    -> client deve aspettare fronte di salita di busy e fronte di discesa di busy, poi vede risultato

    # -----------------------------------------------------------------------------------------------------------------
    # OPC-UA Methods
    # -----------------------------------------------------------------------------------------------------------------
    @opcua.uamethod
    def _run_demo(self, parent, pickAreaId, pickPosX, pickPosY, placeAreaId, placePosX, placePosY):
        #rospy.loginfo('align_base OPC-UA Method called')
        fun = lambda: self._run_demo_method(pickAreaId, pickPosX, pickPosY, placeAreaId, placePosX, placePosY)
        #fun.__name__= '_align_base_method'
        if not self._control_busy_variable.get_value():
            self._queue.put(fun)
            return True
        return False


    # -----------------------------------------------------------------------------------------------------------------
    # Server Methods
    # -----------------------------------------------------------------------------------------------------------------
    def _build_server(self):
        self._server = opcua.Server()
        self._server.set_endpoint("opc.tcp://0.0.0.0:{}/".format(self._port))
        self._server.set_server_name("kuka_ros_demo")
        idx = self._server.register_namespace('http://kuka.com')

        objects = self._server.get_objects_node()
        kuka_object = objects.add_object(opcua.ua.NodeId('Kuka', idx), 'Kuka')

	    # Control Methods
        methods_part = kuka_object.add_folder(opcua.ua.NodeId('Kuka.Control_Methods', idx), 'Control_Methods')

        # Control ready variable
        self._control_busy_variable = methods_part.add_variable(opcua.ua.NodeId('Kuka.Control_Busy', idx),
                                                                'Control_Busy', False,
                                                                varianttype=opcua.ua.VariantType.Boolean)
        self._last_method_result_variable = methods_part.add_variable(
            opcua.ua.NodeId('Kuka.Last_Method_Result', idx), 'Last_Method_Result', False,
            varianttype=opcua.ua.VariantType.Boolean)
        self._method_calls_counter_variable = methods_part.add_variable(
            opcua.ua.NodeId('Kuka.Methods_Call_Counter', idx), 'Methods_Call_Counter', 0,
            varianttype=opcua.ua.VariantType.Int32)
            
            
        inarg_file_name_result_1 = opcua.ua.Argument()
        inarg_file_name_result_1.Name = "pick_areaID"
        inarg_file_name_result_1.DataType = opcua.ua.NodeId(opcua.ua.ObjectIds.Double)
        
        inarg_file_name_result_2 = opcua.ua.Argument()
        inarg_file_name_result_2.Name = "pick_areaX"
        inarg_file_name_result_2.DataType = opcua.ua.NodeId(opcua.ua.ObjectIds.Double)
        
        inarg_file_name_result_3 = opcua.ua.Argument()
        inarg_file_name_result_3.Name = "pick_areaY"
        inarg_file_name_result_3.DataType = opcua.ua.NodeId(opcua.ua.ObjectIds.Double)
        
        
        inarg_file_name_result_4 = opcua.ua.Argument()
        inarg_file_name_result_4.Name = "place_areaID"
        inarg_file_name_result_4.DataType = opcua.ua.NodeId(opcua.ua.ObjectIds.Double)
        
        inarg_file_name_result_5 = opcua.ua.Argument()
        inarg_file_name_result_5.Name = "place_areaX"
        inarg_file_name_result_5.DataType = opcua.ua.NodeId(opcua.ua.ObjectIds.Double)
        
        inarg_file_name_result_6 = opcua.ua.Argument()
        inarg_file_name_result_6.Name = "place_areaY"
        inarg_file_name_result_6.DataType = opcua.ua.NodeId(opcua.ua.ObjectIds.Double)
        
        args_t = [inarg_file_name_result_1,inarg_file_name_result_2,inarg_file_name_result_3,inarg_file_name_result_4,inarg_file_name_result_5,inarg_file_name_result_6]
       
        # /robot/univr/align_base
        kuka_object.add_method(opcua.ua.NodeId('Kuka.demo_pick_place', idx), 'demo_pick_place',
                                self._run_demo, args_t, [opcua.ua.VariantType.Boolean])
       
        self._server.start()
        self._worker_thread.start()

    def start(self):
        self._build_server()
        rospy.loginfo('OPCUA Server running on port {}'.format(self._port))
        self._started = True

    def stop(self):
        if self._started:
            self._server.stop()
            rospy.loginfo('OPCUA Server stopped')
            self._queue.put(False)
            self._worker_thread.join(5)
            self._started = False


def main():
    rospy.init_node('non_blocking_opcua_server')
    s = OpcUAServer(rospy.get_param("port", 4840))
    s.start()
    try:
        rospy.spin()
    except Exception as e:
        pass
    s.stop()


if __name__ == '__main__':
    main()
