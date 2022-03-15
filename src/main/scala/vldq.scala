package vlsu

import chisel3._
import chisel3.util._
import boom.util._

class VLdQueueHandler(ap: VLSUArchitecturalParams) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val brUpdate = Input(new BranchUpdateInfo(ap))
    /** Vector uop from dispatch stage. */
    val vuopDis = Flipped(Vec(ap.coreWidth, Valid(new VLSMicroOP(ap))))
    /** Vector uop from register-read stage with data/address */
    val vuopRR = Flipped(Vec(ap.coreWidth, Valid(new VLSMicroOP(ap))))
    /** Winner req from entries */
    val vldReq = Vec(ap.coreWidth, Decoupled(new VLdRequest(ap)))
    /** Finish ack from wb controller */
    val finishAck = Flipped(Valid(new VLdRequest(ap)))
    /** Tell core pipeline index of newly allocated load queue entry. */
    val disVLdQIdx = Output(Vec(ap.coreWidth, Valid(new DispatchAck(ap))))
    /** Tell core stop dispatch when no entry left. */
    val vLdQFull = Output(Vec(ap.coreWidth, Bool()))
    /** Tell Rob that one vuop has done all elements loads. */
    val toRob = new VLSUROBIO(ap)
    /** Retire finished load queue entry. */
    val fromRob = new ROBVLSUIO(ap)
    /** release done reg for next process. */
    val wakeUp = ValidIO(UInt(ap.nVPregSz.W))

    /** for untouched load, we need to copy original data and write back to new reg. */
    val vrfReadReq = Decoupled(new VLSUReadVRFReq(ap))
    val vrfReadResp = Flipped(Valid(new VLSUReadVRFResp(ap)))
    val vrfWriteReq = Decoupled(new VLSUWriteVRFReq(ap))

    val vrfBusyStatus = Input(UInt(ap.nVRegs.W))
  })
  val nEntries = ap.nVLdQEntries
  val nIdxBits = ap.nVLdQIdxBits
  val headPtr = RegInit(0.U(nIdxBits.W))
  val tailPtr = RegInit(0.U(nIdxBits.W))

  val nonUnitStrideOHs = RegInit(0.U.asTypeOf(Vec(nEntries, Bool())))
  io.toRob := 0.U.asTypeOf(new VLSUROBIO(ap))
  val vldReqArb = Module(new RequestArbitrator(ap, new VLdRequest(ap), true))
  val vuopDisInputs = WireInit(0.U.asTypeOf(Vec(nEntries, Valid(new VLSMicroOP(ap)))))
  val vuopRRInputs = WireInit(0.U.asTypeOf(Vec(nEntries, Valid(new VLSMicroOP(ap)))))

  val reqCandidates = WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(new VLdRequest(ap)))))
  val toRobVec = WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(UInt(ap.robAddrSz.W)))))
  val finishVec = WireInit(0.U.asTypeOf(Vec(nEntries, Bool())))
  val wakeUpVec = WireInit(0.U.asTypeOf(Vec(nEntries, Decoupled(UInt(ap.nVPregSz.W)))))
  val vrfReadArb = Module(new Arbiter(new VLSUReadVRFReq(ap), ap.nVLdQEntries))
  val vrfWriteArb = Module(new Arbiter(new VLSUWriteVRFReq(ap), ap.nVLdQEntries))
  val entries = Seq.tabulate(nEntries){ i =>
    val e = Module(new VLdQEntry(ap, i))
    e.io.vuopDis := vuopDisInputs(i)
    e.io.vuopRR := vuopRRInputs(i)
    reqCandidates(i) <> e.io.uReq
    e.io.finishAck := io.finishAck
    toRobVec(i) <> e.io.robAck
    finishVec(i) := e.io.allDone
    wakeUpVec(i) <> e.io.wakeUp
    e.io.fromRob := io.fromRob
    e.io.headPtr := headPtr
    e.io.tailPrt := tailPtr
    e.io.nonUnitStrideOHs := nonUnitStrideOHs.asUInt()
    vrfReadArb.io.in(i) <> e.io.vrfReadReq
    e.io.vrfReadResp := io.vrfReadResp
    vrfWriteArb.io.in(i) <> e.io.vrfWriteReq
    e.io.vrfBusyStatus := io.vrfBusyStatus
    e.io.brUpdate := io.brUpdate
    e
  }
  io.vrfReadReq <> vrfReadArb.io.out
  io.vrfWriteReq <> vrfWriteArb.io.out
  (vldReqArb.io.inputReq zip reqCandidates).foreach { case (a, req) => a <> req}
  //___________________________________________________________________________//
  //-------------------------Dispatch Allocate---------------------------------//
  val enqPtrVec = Wire(Vec(ap.coreWidth, UInt(nIdxBits.W)))
  for (w <- 0 until ap.coreWidth){
    if (w == 0) enqPtrVec(w) := tailPtr
    else enqPtrVec(w) := WrapInc(enqPtrVec(w - 1), nEntries)
  }
  val loadVlds = io.vuopDis.map(in => in.valid && in.bits.uCtrlSig.accessType.isLoad)
  val loadVldCount = PopCount(loadVlds)
  io.disVLdQIdx.foreach(idx => idx := 0.U.asTypeOf(Valid(new DispatchAck(ap))))
  for (w <- 0 until ap.coreWidth){
    val preVlds = if(w == 0) 0.U else VecInit(loadVlds.slice(0,w)).asUInt()
    val preVldCount = PopCount(preVlds)
    val enqPtr: UInt = enqPtrVec(preVldCount)
    val vLdQFull = WrapInc(enqPtr, ap.nVLdQEntries) === headPtr
    io.vLdQFull(w) := vLdQFull
    val isVectorLoad: Bool = io.vuopDis(w).valid && io.vuopDis(w).bits.uCtrlSig.accessType.isLoad
    val nonUnitStride = isVectorLoad && (io.vuopDis(w).bits.uCtrlSig.accessStyle.isConstantStride ||
      io.vuopDis(w).bits.uCtrlSig.accessStyle.isIndexed)
    when(isVectorLoad){
      vuopDisInputs(enqPtr).valid := true.B
      vuopDisInputs(enqPtr).bits := io.vuopDis(w).bits
      when(nonUnitStride){
        nonUnitStrideOHs(enqPtr) := true.B
      }
      io.disVLdQIdx(w).valid := true.B
      io.disVLdQIdx(w).bits.qIdx := enqPtr
      io.disVLdQIdx(w).bits.robIdx := io.vuopDis(w).bits.robIdx
    }
  }
  when(VecInit(loadVlds).asUInt().orR()){
    tailPtr := WrapInc(enqPtrVec(loadVldCount - 1.U), ap.nVLdQEntries)
  }
  //___________________________________________________________________________//
  //-------------------------Register read-------------------------------------//
  // Each two index should not be same.
  io.vuopRR.foreach{ rr =>
    when(rr.valid && rr.bits.uCtrlSig.accessType.isLoad){
      vuopRRInputs(rr.bits.vLdQIdx) := rr
    }
  }
  //___________________________________________________________________________//
  //-------------------------Request Arbitration-------------------------------//
  (io.vldReq zip vldReqArb.io.outputReq).foreach {case (outReq, arbWinner) =>
    outReq <> arbWinner
  }
  //___________________________________________________________________________//
  //-------------------------Wake up on partial finish-------------------------//
  val wakeUpArbiter = Module(new Arbiter(UInt(ap.nVPregSz.W), nEntries))
  io.wakeUp.valid := wakeUpArbiter.io.out.valid
  io.wakeUp.bits := wakeUpArbiter.io.out.bits
  wakeUpArbiter.io.out.ready := true.B
  (wakeUpVec zip wakeUpArbiter.io.in).foreach {case (entryOut, arbIn) => arbIn <> entryOut}
  //___________________________________________________________________________//
  //-------------------------Ack to rob when finished--------------------------//
  val ackArbiter = Module(new RequestArbitrator(ap, UInt(ap.robAddrSz.W), true))
  ackArbiter.io.inputReq <> toRobVec
  (io.toRob.robIdx zip ackArbiter.io.outputReq).foreach{case (out, ack) =>
    out := ack
    ack.ready := true.B
  }
  //___________________________________________________________________________//
  //-------------------------Dequeue on Commit---------------------------------//
  val finished = finishVec(headPtr)
  when(nonUnitStrideOHs(headPtr) && finished){
    nonUnitStrideOHs(headPtr) := false.B
  }
  headPtr := Mux(finished, WrapInc(headPtr, ap.nVLdQEntries), headPtr)
}

class VLdQEntry(ap: VLSUArchitecturalParams, id: Int) extends VLSUModules(ap){
  val io = IO(new Bundle{
    val brUpdate = Input(new BranchUpdateInfo(ap))
    val entryId = Output(UInt(ap.nVLdQIdxBits.W))
    /** arrived vuop from dispatch stage. */
    val vuopDis = Flipped(Valid(new VLSMicroOP(ap)))
    /** Arrived vuop from register-read stage. */
    val vuopRR = Flipped(Valid(new VLSMicroOP(ap)))
    /** Output request for a single cache line. */
    val uReq = Decoupled(new VLdRequest(ap))

    /** Write back controller tells vld queue finished access. */
    val finishAck = Flipped(Valid(new VLdRequest(ap)))
    /** tess rob that this entry has all finished. */
    val robAck = Decoupled(UInt(ap.robAddrSz.W))
    val allDone = Output(Bool())
    val fromRob = new ROBVLSUIO(ap)
    val headPtr = Input(UInt(ap.nVLdQIdxBits.W))
    val tailPrt = Input(UInt(ap.nVLdQIdxBits.W))
    val nonUnitStrideOHs = Input(UInt(ap.nVLdQEntries.W))
    /** wake up core pipeline when single register is all done. */
    val wakeUp = Decoupled(UInt(ap.nVPregSz.W))

    val vrfReadReq = Decoupled(new VLSUReadVRFReq(ap))
    val vrfReadResp = Flipped(Valid(new VLSUReadVRFResp(ap)))
    val vrfWriteReq = Decoupled(new VLSUWriteVRFReq(ap))
    val vrfBusyStatus = Input(UInt(ap.nVRegs.W))
  })

  io.uReq.valid := false.B
  io.uReq.bits := 0.U.asTypeOf(new VLdRequest(ap))
  io.robAck.valid := false.B
  io.robAck.bits := 0.U
  io.wakeUp.valid := false.B
  io.wakeUp.bits := 0.U
  io.vrfReadReq.valid := false.B
  io.vrfReadReq.bits := 0.U.asTypeOf(new VLSUReadVRFReq(ap))
  io.vrfWriteReq.valid := false.B
  io.vrfWriteReq.bits := 0.U.asTypeOf(new VLSUWriteVRFReq(ap))
  /** This register record which req buffer entry hold our request.*/
  val reg = RegInit(0.U.asTypeOf(Valid(new VLdQEntryBundle(ap))))

  io.entryId := id.U

  val sIdle:: sWaitRs :: sCopyStale :: sSplitting :: sWaitData :: allDone :: sWaitRetire :: Nil = Enum(7)
  val state: UInt = RegInit(sIdle)
  val updateBrMask = Mux(io.vuopRR.valid, io.vuopRR.bits.brMask, reg.bits.brMask)
  val isKilledByBranch = IsKilledByBranch(io.brUpdate, updateBrMask)
  val isIdle = state === sIdle
  /** Indicates this entry can be dequeued. */
  io.allDone := isIdle && (reg.bits.allSucceeded || isKilledByBranch)

  val isUnitStride = reg.bits.style.isUnitStride
  val isSegment = reg.bits.style.isSegment
  val isIndexed = reg.bits.style.isIndexed

  val snippetVMAdjuster = Module(new SnippetVectorMaskAdjuster(ap))
  snippetVMAdjuster.io.ctrl := io.vuopRR.bits.uCtrlSig.accessStyle
  snippetVMAdjuster.io.vm := io.vuopRR.bits.vm

  val vlAdjust = Module(new SnippetInitializer(ap))
  vlAdjust.io.ctrl := io.vuopRR.bits.uCtrlSig.accessStyle

  val indexedFilter = Module(new SnippetIndexedSplitsFilter(ap))
  indexedFilter.io.ctrl := io.vuopRR.bits.uCtrlSig.accessStyle

  /** If the oldest un-commit non-unit-stride is older than us, hang. */
  val oldestNonUnitStrideIdx = PickOldest(io.nonUnitStrideOHs, io.headPtr, io.tailPrt, ap.nVLdQEntries)
  val freeze = oldestNonUnitStrideIdx.valid && IsOlder(AgePriorityEncoder(io.nonUnitStrideOHs.asBools(), io.headPtr),
    id.U, io.headPtr)

  val requestSpliter = Module(new RequestSpliter(ap, isLoad = true, id = id))
  requestSpliter.io.reg := reg.bits
  io.uReq.bits := requestSpliter.io.uReq.bits
  // Claim a new entry when dispatch.
  when(isIdle){
    when(io.vuopDis.valid){
      reg.bits.addr := 0.U
      reg.bits.preAddr := 0.U
      reg.bits.rs2 := 0.U

      reg.bits.robIndex := io.vuopDis.bits.robIdx
      reg.bits.orderFail := false.B
      reg.bits.allSucceeded := false.B
      reg.bits.pRegVec := io.vuopDis.bits.vpdst
      reg.bits.style := io.vuopDis.bits.uCtrlSig.accessStyle
      reg.bits.reqCount := 0.U
      reg.bits.segmentCount := Mux(io.vuopDis.bits.uCtrlSig.accessStyle.isIndexed, io.vuopDis.bits.uCtrlSig.accessStyle.fieldIdx, 0.U)
      reg.bits.brMask := GetNewBranchMask(io.brUpdate, io.vuopDis.bits.brMask)
      state := sWaitRs
    }
  }.elsewhen(state === sWaitRs){
    when(io.vuopRR.valid){
      reg.bits.addr := io.vuopRR.bits.rs1
      /** indicates if addr is aligned to line address. */
      val offset: UInt = io.vuopRR.bits.rs1(ap.offsetBits - 1, 0)
      val alignedAddr: Bool = offset === 0.U
      val isUnitStride = reg.bits.style.isUnitStride || reg.bits.style.isWholeAccess
      reg.bits.brMask := GetNewBranchMask(io.brUpdate, io.vuopRR.bits.brMask)
      reg.bits.wakeUpVec := VecInit(vlAdjust.io.wakeVecInit.asBools())
      reg.bits.totalSegment := vlAdjust.io.totalSegment
      reg.bits.totalReq := Mux(alignedAddr && isUnitStride, (ap.maxReqsInUnitStride - 1).U, vlAdjust.io.totalRequest)
      reg.bits.preAddr := io.vuopRR.bits.rs1
      reg.bits.vs1 := io.vuopRR.bits.vs1
      reg.bits.vs2 := io.vuopRR.bits.vs2
      reg.bits.rs2 := io.vuopRR.bits.rs2
      reg.bits.staleRegIdxVec := io.vuopRR.bits.staleRegIdxes
      reg.bits.finishMasks :=
        (snippetVMAdjuster.io.adjusterSnippet zip vlAdjust.io.initSnippet).zip(indexedFilter.io.filteredSnippet).map {
          case ((vm, snippet), filtered) => vm | snippet | filtered
        }
      val needFetchTail = !io.vuopRR.bits.uCtrlSig.accessStyle.vta
      val needFetchMask = !io.vuopRR.bits.uCtrlSig.accessStyle.vma
      /** Fetch all if necessary. */
      val needCopyStale = Mux(io.vuopRR.bits.uCtrlSig.accessStyle.isWholeAccess, false.B, needFetchTail || needFetchMask)
      when(needCopyStale){
        state := sCopyStale
      }.otherwise{
        state := sSplitting
      }
    }
  }.elsewhen(state === sCopyStale) {
    val indexEew = reg.bits.style.indexEew
    val dataEew = reg.bits.style.dataEew
    val largerData = dataEew > indexEew
    val dataExpandRate = dataEew - indexEew
    val largerIndex = indexEew > dataEew
    val dataShrinkRate = indexEew - dataEew
    val equal = indexEew === dataEew
    val indexSegIdx = reg.bits.style.fieldIdx
    /** Indicates how many data Regs are affected by this split of vs2. */
    val nActiveDataReg = Mux(largerIndex || equal, 1.U, 1.U << dataExpandRate)
    val startDataRegIdx = Mux(largerIndex, (indexSegIdx >> dataShrinkRate).asUInt(),
      Mux(equal, indexSegIdx, (indexSegIdx << dataExpandRate).asUInt()))
    val startRegIdx = Mux(isIndexed, startDataRegIdx, 0.U)
    val fieldCounter = RegInit(0.U(4.W))
    val endRegIdx = Mux(isIndexed, startRegIdx + nActiveDataReg, Mux(reg.bits.style.vlmul(2), 1.U, (1.U << reg.bits.style.vlmul(1, 0)).asUInt()))
    val staleDataReg = RegInit(0.U.asTypeOf(Valid(UInt(ap.vLen.W))))
    val activeRegIdx = fieldCounter + startRegIdx
    val requestingRegIdx = reg.bits.staleRegIdxVec(activeRegIdx)
    val readSent = RegNext(io.vrfReadReq.fire())
    val copyDone = activeRegIdx === endRegIdx
    io.vrfReadReq.valid := !staleDataReg.valid && !io.vrfBusyStatus(requestingRegIdx) && !readSent && !copyDone
    io.vrfReadReq.bits.addr := requestingRegIdx

    when(io.vrfReadResp.valid && readSent) {
      staleDataReg.valid := true.B
      staleDataReg.bits := io.vrfReadResp.bits.data
    }
    val affectPartialDataReg = isIndexed && largerIndex
    val shrinkHalf = dataShrinkRate === 1.U
    val shrinkQuarter = dataShrinkRate === 2.U
    val shrinkOctant = dataShrinkRate === 3.U
    val partialMaskHalf = VecInit(UIntToOH(indexSegIdx(0), 2).asBools().map(b => Fill(ap.vLenb / 2, b))).asUInt()
    val partialMaskQuarter = VecInit(UIntToOH(indexSegIdx(1, 0), 4).asBools().map(b => Fill(ap.vLenb / 4, b))).asUInt()
    val partialMaskOctant = VecInit(UIntToOH(indexSegIdx, 8).asBools().map(b => Fill(ap.vLenb / 8, b))).asUInt()
    val partialMask = Mux(shrinkHalf, partialMaskHalf, Mux(shrinkQuarter, partialMaskQuarter, partialMaskOctant))
    val staleActiveMask = Mux(affectPartialDataReg, partialMask, Fill(ap.vLenb, 1.U(1.W)))
    io.vrfWriteReq.valid := staleDataReg.valid
    io.vrfWriteReq.bits.addr := reg.bits.pRegVec(activeRegIdx)
    io.vrfWriteReq.bits.data := staleDataReg.bits
    io.vrfWriteReq.bits.byteMask := staleActiveMask
    when(io.vrfWriteReq.fire()) {
      fieldCounter := fieldCounter + 1.U
      staleDataReg.valid := false.B
    }
    when(copyDone) {
      state := sSplitting
      staleDataReg.valid := false.B
      fieldCounter := 0.U
    }
  }.elsewhen(state === sSplitting){
    io.uReq.valid := requestSpliter.io.uReq.valid && !freeze && !isKilledByBranch
    io.uReq.bits := requestSpliter.io.uReq.bits
    /** Ready is true only when valid is true, but unnecessary request does not trigger valid. Both shift to next. */
    val nextSplit: Bool = (io.uReq.ready || !io.uReq.valid) && !freeze
    val nextSeg: Bool = reg.bits.reqCount === reg.bits.totalReq - 1.U
    reg.bits.reqCount := Mux(nextSplit, Mux(nextSeg, 0.U, reg.bits.reqCount + 1.U), reg.bits.reqCount)
    reg.bits.preAddr := Mux(nextSplit, Mux(nextSeg, reg.bits.addr, requestSpliter.io.newAddr), reg.bits.preAddr)
    reg.bits.preSnippet := Mux(nextSplit, Mux(nextSeg, 0.U, requestSpliter.io.newSnippet), reg.bits.preSnippet)
    reg.bits.segmentCount := Mux(nextSplit && nextSeg, reg.bits.segmentCount + 1.U, reg.bits.segmentCount)
    val splitDone = (reg.bits.segmentCount === (reg.bits.totalSegment - 1.U)) && nextSeg && nextSplit
    when(splitDone){state := sWaitData}
    when(io.finishAck.valid && io.finishAck.bits.qEntryIdx === id.U){
      reg.bits.finishMasks(io.finishAck.bits.segmentIdx) := reg.bits.finishMasks(io.finishAck.bits.segmentIdx) | io.finishAck.bits.regAccessCS.finishMaskSnippet
    }
  }.elsewhen(state === sWaitData){
    when(io.finishAck.valid && io.finishAck.bits.qEntryIdx === id.U){
      reg.bits.finishMasks(io.finishAck.bits.segmentIdx) := reg.bits.finishMasks(io.finishAck.bits.segmentIdx) | io.finishAck.bits.regAccessCS.finishMaskSnippet
    }
    /** indicates we have some vreg to wake, 1 is valid. */
    val needWakeUpVec = VecInit((reg.bits.finishMasks zip reg.bits.wakeUpVec).map { case (finish, wake) =>
      wake && finish.andR()}).asUInt()
    val needWakeUp = needWakeUpVec.orR()
    /** indicates which reg in this group need wake up, not the reg addr. */
    val wakeUpGroupIdx = OHToUInt(FindFirstOne(needWakeUpVec, 8))
    val wakeUpRegIdx: UInt = reg.bits.pRegVec(wakeUpGroupIdx)
    when(needWakeUp){
      io.wakeUp.valid := true.B
      io.wakeUp.bits := wakeUpRegIdx
      when(io.wakeUp.fire()){
        reg.bits.wakeUpVec(wakeUpRegIdx) := false.B
      }
    }
    val allFinished = !reg.bits.wakeUpVec.asUInt().orR()
    when(allFinished){
      io.robAck.valid := true.B
      io.robAck.bits := reg.bits.robIndex
      when(io.robAck.fire()){
        state := sWaitRetire
      }
    }
  }.elsewhen(state === sWaitRetire){
    //we have tell rob that we have all data written, which means vec pipe may use our data.
    //If there is a order fail. pipeline has to be flushed.
    when(VecInit(io.fromRob.retireEntries.map{i => i.valid && i.bits.isLoad && i.bits.qEntryIdx === id.U}).asUInt().orR()){
      state := sIdle
      reg.bits.allSucceeded := true.B
      reg.valid := false.B
    }
  }
  when(state =/= sIdle && isKilledByBranch){
    state := sIdle
    reg.bits.allSucceeded := true.B
  }
  when(io.vuopDis.valid) {assert(isIdle, "allocating a busy entry.")}
}